// wl_base_top : 이벤트마다 입력을 받는(XYPT 방식) 기준 설계. wl_top 과
// 견주는 데 쓴다.
//
// 보통의 이벤트 파이프라인은 이벤트마다 그 좌표와 시각을 함께 받으므로, 회전각과 그
// cos / sin 을 이벤트마다 구해야 하고 곱 네 개도 이벤트마다 만든다:
//     u = C*dx - S*dy + WC*2^14,  v = S*dx + C*dy + WC*2^14,  wx = u >>> 16, wy = v >>> 16
// 여기서 dx = x - cx, dy = y - cy 이고 C, S 는 소수 14 비트를 가진 부호 있는 16비트라,
// 16 만큼의 시프트가 소수를 버리면서 칸 크기 4 px 로 나눈다(wl_xform 과 같다. 다만
// wl_xform 에서는 곱을 열마다 · 그룹마다 나눠 쓴다).
// 여기서는 이벤트마다 (x, y, phase) 로 들어온다: 그 frame 의 phase 가 이벤트와 함께
// 주어지므로, 실제 XYPT 입력단이라면 더 필요할 timestamp -> 각도 보간이 빠진다
// (이 기준 설계에 유리한 쪽이다). wl_cordic 의 CORDIC 을 16 단 파이프라인으로 펼쳐
// 클록마다 이벤트 하나가 들어갈 수 있게 했다. 월드 칸을 구한 다음부터는 모두 wl_top 과
// 같다: 같은 원 표(wl_region_map)와 같은 월드 메모리 블록(병합과 single-port bank
// 16 개를 갖춘 wl_accum). 그래서 두 top 을 견주면 입력 형식만의 비용이 드러난다.
//
// 포트는 입력을 빼면 wl_top 과 같다: in_x, in_y, in_phase / in_valid /
// in_ready 가 MGROUP word 대신 전송마다 이벤트 하나를 나르고, 따로 된 phase 입력과
// frame counter 는 없다. drop_cnt 는 칸이 저장되지 않는 이벤트를 센다. clr 와
// 읽기 포트는 wl_top 과 같이 동작한다.
//
// 파이프라인(클록마다 이벤트 하나, wl_accum 이 멈추면 모든 단이 함께 멈춘다)
//   I        x, y, phase 를 받는다; dx = x - cx, dy = y - cy; CORDIC 시작 벡터
//   C1..C16  단마다 CORDIC 반복 하나(wl_cordic 과 비트 단위로 같다)
//   F        guard bit 를 반올림하고 사분면만큼 돌린다 -> C, S
//   M        곱 네 개 C*dx, S*dy, S*dx, C*dy
//   U        u, v
//   E        wx, wy, 정사각형 · 원 검사 -> wl_accum
module wl_base_top #(
    parameter N    = 352,
    parameter M    = 352,
    parameter BITS = 13,
    parameter CW   = 4,
    parameter WC   = 704
) (
    input  wire            clk,
    input  wire            rst_n,

    input  wire      [9:0] cx,
    input  wire      [9:0] cy,

    input  wire      [9:0] in_x,
    input  wire      [9:0] in_y,
    input  wire     [15:0] in_phase,
    input  wire            in_valid,
    output wire            in_ready,

    input  wire            clr,
    output wire            clr_busy,

    input  wire            rd_en,
    input  wire      [8:0] rd_x,
    input  wire      [8:0] rd_y,
    output wire [BITS-1:0] rd_data,

    output wire            idle,
    output reg      [23:0] drop_cnt
);
    localparam signed [18:0] X0  = 19'sd56281;      // CORDIC 시작 벡터, wl_cordic 과 같다
    localparam signed [26:0] OFF = WC * 16384;      // WC*2^14, wl_xform 과 같다

    // ATAN[i] = round(atan(2^-i) / 2pi * 2^19), wl_cordic 과 같다.
    function [16:0] atan(input integer i);
        case (i)
            0:  atan = 17'd65536;
            1:  atan = 17'd38688;
            2:  atan = 17'd20442;
            3:  atan = 17'd10377;
            4:  atan = 17'd5208;
            5:  atan = 17'd2607;
            6:  atan = 17'd1304;
            7:  atan = 17'd652;
            8:  atan = 17'd326;
            9:  atan = 17'd163;
            10: atan = 17'd81;
            11: atan = 17'd41;
            12: atan = 17'd20;
            13: atan = 17'd10;
            14: atan = 17'd5;
            default: atan = 17'd3;
        endcase
    endfunction

    wire        ev_ready, acc_busy;
    reg         ev_valid;
    reg   [8:0] ev_x, ev_y;
    wire        adv = !ev_valid || ev_ready;     // 모든 단이 함께 넘어간다

    // ---------------------------------------------------------------- I, C1..C16
    // 첨자 0 은 단 I, 첨자 k 는 단 Ck(반복 k - 1 뒤)다.
    reg                cv [0:16];
    reg  signed [18:0] cx_ [0:16];
    reg  signed [18:0] cy_ [0:16];
    reg  signed [17:0] cz [0:16];
    reg          [1:0] cq [0:16];
    reg  signed [10:0] cdx [0:16];
    reg  signed [10:0] cdy [0:16];

    assign in_ready = adv && !clr_busy;

    // ---------------------------------------------------------------- F, M, U, E
    reg                f_v, m_v, u_v;
    reg  signed [15:0] f_c, f_s;
    reg  signed [10:0] f_dx, f_dy, m_dx, m_dy;
    reg  signed [25:0] p_cx, p_sy, p_sx, p_cy;
    reg  signed [26:0] u, v;

    wire signed [18:0] xr = (cx_[16] + 19'sd4) >>> 3;
    wire signed [18:0] yr = (cy_[16] + 19'sd4) >>> 3;
    wire signed [15:0] xo = xr[15:0];
    wire signed [15:0] yo = yr[15:0];

    wire signed [10:0] wx = u[26:16];
    wire signed [10:0] wy = v[26:16];
    wire               in_sq = !wx[10] && (wx < N) && !wy[10] && (wy < M);
    wire               in_map;
    wire        [12:0] map_a;           // 여기서는 쓰지 않는다. wl_accum 이 word 를 다시 찾는다
    wl_region_map u_map (.wx(wx[8:0]), .wy(wy[8:0]), .hit(in_map), .addr(map_a));

    reg                any_v;
    integer            k;
    always @(*) begin
        any_v = f_v || m_v || u_v || ev_valid;
        for (k = 0; k <= 16; k = k + 1) any_v = any_v || cv[k];
    end
    assign idle = !any_v && !acc_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k <= 16; k = k + 1) begin
                cv[k]  <= 1'b0;
                cx_[k] <= 19'sd0;
                cy_[k] <= 19'sd0;
                cz[k]  <= 18'sd0;
                cq[k]  <= 2'd0;
                cdx[k] <= 11'sd0;
                cdy[k] <= 11'sd0;
            end
            f_v      <= 1'b0;
            m_v      <= 1'b0;
            u_v      <= 1'b0;
            f_c      <= 16'sd0;
            f_s      <= 16'sd0;
            f_dx     <= 11'sd0;
            f_dy     <= 11'sd0;
            m_dx     <= 11'sd0;
            m_dy     <= 11'sd0;
            p_cx     <= 26'sd0;
            p_sy     <= 26'sd0;
            p_sx     <= 26'sd0;
            p_cy     <= 26'sd0;
            u        <= 27'sd0;
            v        <= 27'sd0;
            ev_valid <= 1'b0;
            ev_x     <= 9'd0;
            ev_y     <= 9'd0;
            drop_cnt <= 24'd0;
        end else if (adv) begin
            // I
            cv[0]  <= in_valid && !clr_busy;
            cx_[0] <= X0;
            cy_[0] <= X0;
            cz[0]  <= $signed({1'b0, in_phase[13:0], 3'b000}) - 18'sd65536;
            cq[0]  <= in_phase[15:14];
            cdx[0] <= $signed({1'b0, in_x}) - $signed({1'b0, cx});
            cdy[0] <= $signed({1'b0, in_y}) - $signed({1'b0, cy});

            // C1..C16: 반복 k - 1
            for (k = 1; k <= 16; k = k + 1) begin
                cv[k]  <= cv[k-1];
                cq[k]  <= cq[k-1];
                cdx[k] <= cdx[k-1];
                cdy[k] <= cdy[k-1];
                if (!cz[k-1][17]) begin
                    cx_[k] <= cx_[k-1] - (cy_[k-1] >>> (k - 1));
                    cy_[k] <= cy_[k-1] + (cx_[k-1] >>> (k - 1));
                    cz[k]  <= cz[k-1] - $signed({1'b0, atan(k - 1)});
                end else begin
                    cx_[k] <= cx_[k-1] + (cy_[k-1] >>> (k - 1));
                    cy_[k] <= cy_[k-1] - (cx_[k-1] >>> (k - 1));
                    cz[k]  <= cz[k-1] + $signed({1'b0, atan(k - 1)});
                end
            end

            // F
            f_v  <= cv[16];
            f_dx <= cdx[16];
            f_dy <= cdy[16];
            case (cq[16])
                2'd0:    begin f_c <=  xo; f_s <=  yo; end
                2'd1:    begin f_c <= -yo; f_s <=  xo; end
                2'd2:    begin f_c <= -xo; f_s <= -yo; end
                default: begin f_c <=  yo; f_s <= -xo; end
            endcase

            // M
            m_v  <= f_v;
            p_cx <= f_c * f_dx;
            p_sy <= f_s * f_dy;
            p_sx <= f_s * f_dx;
            p_cy <= f_c * f_dy;

            // U
            u_v <= m_v;
            u   <= p_cx - p_sy + OFF;
            v   <= p_sx + p_cy + OFF;

            // E
            ev_valid <= u_v && in_sq && in_map;
            ev_x     <= wx[8:0];
            ev_y     <= wy[8:0];
            if (u_v && !(in_sq && in_map)) drop_cnt <= drop_cnt + 24'd1;
        end
    end

    wl_accum #(.N(N), .M(M), .BITS(BITS), .CW(CW)) u_acc (
        .clk      (clk),
        .rst_n    (rst_n),
        .ev_valid (ev_valid),
        .ev_ready (ev_ready),
        .ev_x     (ev_x),
        .ev_y     (ev_y),
        .clr      (clr),
        .clr_on   (clr_busy),
        .rd_en    (rd_en),
        .rd_x     (rd_x),
        .rd_y     (rd_y),
        .rd_data  (rd_data),
        .busy     (acc_busy)
    );
endmodule
