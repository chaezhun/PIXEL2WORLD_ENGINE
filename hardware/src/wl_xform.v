// wl_xform : 작업 -> 월드 메모리 칸, 클럭마다 이벤트 하나.
//
// 각 이벤트 (x, y) 를 회전 중심 (cx, cy) 둘레로 프레임 각도만큼 돌리고, 회전 중심이
// 월드 중심 WC 에 오도록 옮긴다:
//     u = C*dx - S*dy + WC*2^14,  v = S*dx + C*dy + WC*2^14,  wx = u >>> 16, wy = v >>> 16
// 여기서 dx = x - cx, dy = y - cy 이고, C, S 는 프레임 각도의 cos / sin 을 소수 14비트의
// 부호 있는 16비트 수로 나타낸 값이다(1.0 = 16384). 따라서 u 와 v 는 소수 14비트를 가진
// 월드 px 이고, 16비트 산술 시프트가 소수부를 버리면서(내림) 4 px 칸 크기로 나눈다.
//
// MGROUP 스트림에서 x 는 다음 열 주소까지 모든 이벤트가 함께 쓰고, y = y0 + k 에서
// y0 = 8 * group 은 한 그룹의 (최대 8개) 이벤트가 함께 쓴다. 그래서 곱을 나눠 구한다:
//     Xu = C*(x - cx) + WC*2^14            열마다 한 번      (COL 작업)
//     Xv = S*(x - cx) + WC*2^14
//     Bu = Xu - S*(y0 - cy)                그룹마다 한 번    (GRP 작업)
//     Bv = Xv + C*(y0 - cy)
//     u  = Bu - k*S,  v = Bv + k*C         이벤트마다, k = 0..7, 시프트와 덧셈만
// 같은 정수 합을 순서만 바꿔 더한 것이라 결과는 이벤트별 식과 비트까지 같다.
// 16 x 11비트 부호 있는 곱셈기 두 개가 COL 과 GRP 작업을 함께 맡고, 이벤트별 단계는
// 덧셈기만 쓴다.
//
// 단계
//   S1  곱셈       : wl_decode 에서 온 작업, P1/P2 = (C 또는 S) * (x - cx 또는 y0 - cy).
//                    COL 작업은 한 클럭 뒤 Xu / Xv 를 쓰고 나간다. GRP 작업은
//                    X 가 받을 수 있을 때까지 S1 에서 기다린다.
//   X   펼치기     : 그룹 하나를 잡고, 켜진 행을 클럭마다 하나씩 낸다(낮은 행부터)
//   E1  회전       : 낸 행의 u, v
//   E2  칸         : wx, wy, 범위 검사: N x M 격자 안이고 wl_region_map 의 저장 영역
//                    표가 저장하는 칸이어야 한다. 나머지는 버리고 drop_cnt 에
//                    센다
//
// 멈춤: 병합한 갱신이 바쁜 뱅크를 기다려야 할 때 wl_accum 은 한 클럭 동안 이벤트를
// 거절할 수 있다(ev_ready 0). 그러면 X, E1, E2 가 함께 멈추고, S1 은 그룹을 X 에 넘기지
// 못한다. X 앞의 작업은 보통의 ready 사슬로 멈춘다.
//
// 각도 처리: cur_* 는 지금 S1 에 들어오는 프레임의 (C, S), nxt_* 는 다음 프레임의 값으로,
// wl_cordic 이 위상 입력에서 미리 구한다. BND 작업이 cur <- nxt 로 바꾼다. 모든 작업은
// cur 가 유효할 때까지 기다리므로 프레임이 지난 각도로 처리되는 일은 없다. 다음 위상이 아직
// 오지 않았으면 cur 는 그것이 올 때까지 무효가 된다. 그룹마다 자기 (C, S) 를 X 로 가져가므로,
// 아직 펼치는 중인 이전 프레임의 그룹은 이 전환의 영향을 받지 않는다.
module wl_xform #(
    parameter N  = 352,     // 월드 메모리 열 수
    parameter M  = 352,     // 월드 메모리 행 수
    parameter WC = 704      // 월드 중심(px, N * 4 / 2), (cx, cy) 가 오는 자리
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire  [9:0] cx,          // 회전 중심, px (고정)
    input  wire  [9:0] cy,

    input  wire [15:0] ph_data,     // 프레임별 위상, k 번째 프레임에 k 번째 위상
    input  wire        ph_valid,
    output wire        ph_ready,

    input  wire        j_valid,
    output wire        j_ready,
    input  wire  [1:0] j_type,
    input  wire  [9:0] j_val,
    input  wire  [7:0] j_map,

    output reg         ev_valid,    // 범위 안의 이벤트 하나, 칸 (ev_x, ev_y)
    input  wire        ev_ready,
    output reg   [8:0] ev_x,
    output reg   [8:0] ev_y,
    output reg  [23:0] drop_cnt,    // 칸이 저장되지 않는 이벤트 수(2^24 에서 0 으로 돌아간다)

    output wire        busy
);
    localparam [1:0] J_COL = 2'd0, J_GRP = 2'd1, J_BND = 2'd2;
    localparam signed [26:0] OFF = WC * 16384;     // WC*2^14: 소수 14비트를 붙인 월드 중심

    // ---------------------------------------------------------------- 각도
    reg               cur_ok, nxt_ok;
    reg signed [15:0] cur_c, cur_s, nxt_c, nxt_s;
    wire              cd_busy, cd_done;
    wire signed [15:0] cd_c, cd_s;

    // cd_done 인 클럭은 뺀다: 그 클럭에 결과를 nxt 에 쓴다.
    assign ph_ready = !nxt_ok && !cd_busy && !cd_done;

    wl_cordic u_cordic (
        .clk   (clk),
        .rst_n (rst_n),
        .start (ph_valid && ph_ready),
        .phase (ph_data),
        .busy  (cd_busy),
        .done  (cd_done),
        .cos_o (cd_c),
        .sin_o (cd_s)
    );

    // ---------------------------------------------------------------- S1
    reg               s1_v;
    reg         [1:0] s1_t;
    reg         [7:0] s1_map;
    reg signed [25:0] p1, p2;
    reg signed [26:0] xu, xv;

    wire              x_ready;
    wire              s1_take = s1_v && (s1_t == J_GRP) && x_ready;   // 그룹이 X 로 넘어간다
    wire              s1_free = !s1_v || (s1_t == J_COL) || s1_take;
    assign j_ready = s1_free && cur_ok;
    wire              j_fire  = j_valid && j_ready;

    // 곱셈기 피연산자. COL: (C, S) * (x - cx). GRP: (S, C) * (y0 - cy).
    wire              is_col = (j_type == J_COL);
    wire signed [10:0] d     = $signed({1'b0, j_val}) - $signed({1'b0, is_col ? cx : cy});
    wire signed [15:0] m1    = is_col ? cur_c : cur_s;
    wire signed [15:0] m2    = is_col ? cur_s : cur_c;

    // ---------------------------------------------------------------- X
    reg               xq_v;
    reg         [7:0] xq_map;
    reg signed [26:0] bu, bv;
    reg signed [15:0] xq_c, xq_s;

    wire        [7:0] map_rest = xq_map & (xq_map - 8'd1);   // 가장 낮은 1 비트를 지운다
    wire              x_last   = (map_rest == 8'd0);
    wire              adv      = !ev_valid || ev_ready;      // X, E1, E2 가 다음으로 나아간다
    assign x_ready = adv && (!xq_v || x_last);
    reg         [2:0] k;
    always @(*) begin
        casez (xq_map)
            8'b???????1: k = 3'd0;
            8'b??????10: k = 3'd1;
            8'b?????100: k = 3'd2;
            8'b????1000: k = 3'd3;
            8'b???10000: k = 3'd4;
            8'b??100000: k = 3'd5;
            8'b?1000000: k = 3'd6;
            default:     k = 3'd7;
        endcase
    end

    // k*S 와 k*C 를 시프트와 덧셈으로 구한다(k <= 7).
    wire signed [18:0] ks = (k[0] ? {{3{xq_s[15]}}, xq_s} : 19'sd0)
                          + (k[1] ? {{2{xq_s[15]}}, xq_s, 1'b0} : 19'sd0)
                          + (k[2] ? {xq_s[15], xq_s, 2'b00} : 19'sd0);
    wire signed [18:0] kc = (k[0] ? {{3{xq_c[15]}}, xq_c} : 19'sd0)
                          + (k[1] ? {{2{xq_c[15]}}, xq_c, 1'b0} : 19'sd0)
                          + (k[2] ? {xq_c[15], xq_c, 2'b00} : 19'sd0);

    // ---------------------------------------------------------------- E1, E2
    reg               e1_v;
    reg signed [26:0] u, v;
    wire signed [10:0] wx = u[26:16];       // u >>> 16
    wire signed [10:0] wy = v[26:16];
    // N x M 격자 안: 음수가 아니고 N / M 보다 작다. 이때만 아래 9비트가
    // 저장 영역 표에 쓸 수 있는 칸 좌표다.
    wire              in_sq  = !wx[10] && (wx < N) && !wy[10] && (wy < M);
    wire              in_map;
    wire       [12:0] map_a;        // 여기서는 쓰지 않는다. wl_accum 이 워드를 다시 찾는다
    wl_region_map u_map (.wx(wx[8:0]), .wy(wy[8:0]), .hit(in_map), .addr(map_a));
    wire              in_rng = in_sq && in_map;

    assign busy = s1_v || xq_v || e1_v || ev_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_ok   <= 1'b0;
            nxt_ok   <= 1'b0;
            cur_c    <= 16'sd0;
            cur_s    <= 16'sd0;
            nxt_c    <= 16'sd0;
            nxt_s    <= 16'sd0;
            s1_v     <= 1'b0;
            s1_t     <= J_COL;
            s1_map   <= 8'd0;
            p1       <= 26'sd0;
            p2       <= 26'sd0;
            xu       <= 27'sd0;
            xv       <= 27'sd0;
            xq_v     <= 1'b0;
            xq_map   <= 8'd0;
            bu       <= 27'sd0;
            bv       <= 27'sd0;
            xq_c     <= 16'sd0;
            xq_s     <= 16'sd0;
            e1_v     <= 1'b0;
            u        <= 27'sd0;
            v        <= 27'sd0;
            ev_valid <= 1'b0;
            ev_x     <= 9'd0;
            ev_y     <= 9'd0;
            drop_cnt <= 24'd0;
        end else begin
            // 각도 레지스터
            if (cd_done) begin
                nxt_c  <= cd_c;
                nxt_s  <= cd_s;
                nxt_ok <= 1'b1;
            end
            if (j_fire && j_type == J_BND) begin
                if (nxt_ok) begin
                    cur_c  <= nxt_c;
                    cur_s  <= nxt_s;
                    nxt_ok <= 1'b0;
                end else begin
                    cur_ok <= 1'b0;
                end
            end else if (!cur_ok && nxt_ok) begin
                cur_c  <= nxt_c;
                cur_s  <= nxt_s;
                cur_ok <= 1'b1;
                nxt_ok <= 1'b0;
            end

            // S1 -> 열 레지스터 (COL 은 한 클럭 뒤 S1 을 떠난다)
            if (s1_v && s1_t == J_COL) begin
                xu <= p1 + OFF;
                xv <= p2 + OFF;
            end
            if (s1_free) s1_v <= 1'b0;
            if (j_fire && j_type != J_BND) begin
                s1_v   <= 1'b1;
                s1_t   <= j_type;
                s1_map <= j_map;
                p1     <= m1 * d;
                p2     <= m2 * d;
            end

            // X
            if (xq_v && adv) begin
                xq_map <= map_rest;
                if (x_last) xq_v <= 1'b0;
            end
            if (s1_take) begin
                xq_v   <= 1'b1;
                xq_map <= s1_map;
                bu     <= xu - p1;
                bv     <= xv + p2;
                xq_c   <= cur_c;
                xq_s   <= cur_s;
            end

            if (adv) begin
                // E1
                e1_v <= xq_v;
                if (xq_v) begin
                    u <= bu - ks;
                    v <= bv + kc;
                end

                // E2
                ev_valid <= e1_v && in_rng;
                ev_x     <= wx[8:0];
                ev_y     <= wy[8:0];
                if (e1_v && !in_rng) drop_cnt <= drop_cnt + 24'd1;
            end
        end
    end
endmodule
