// wl_accum : 병합한 read-modify-write, 포화 카운터, 지우기와 읽기 포트를 갖춘
// 월드 메모리.
//
// 메모리 구성
//   N x M 칸을 각 칸 좌표의 아래 2비트로 단일 포트(1RW) 뱅크 16개에 나눈다.
//   bank = {wy[1:0], wx[1:0]}, 곧 격자를 4 x 4 로 칠한 것이다. 대각선까지 포함해
//   맞닿은 두 칸과, 두 칸 떨어진 두 칸은 언제나 서로 다른 뱅크에 있다. 이중 포트(1R1W)
//   메모리라면 한 갱신의 읽기와 앞 갱신의 쓰기를 같은 클럭에 할 수 있지만, 같은 용량에
//   SRAM 면적이 대략 두 배 든다. 단일 포트 뱅크는 단일 포트 면적으로 처리율을 거의
//   그대로 지킨다.
//   뱅크 안에서 칸은 88 x 88 격자의 행 wy >> 2, 열 wx >> 2 에 놓인다.
//   카메라 roll 로 닿을 수 있는 칸만 저장한다: 모든 픽셀은 월드 중심에 놓이는 회전
//   중심 둘레로 돌므로, 쓰이는 곳은 월드 중심 둘레의 원 안뿐이다. wl_region_map 이 그
//   원의 행별 표로 뱅크 안에서 칸의 워드를 정하며, 뱅크마다 (N / 4) * (M / 4) = 7744
//   대신 BD = 6132 워드다. 표 밖의 이벤트는 wl_xform 이 이미 버렸고, 읽기 포트는
//   거기서 0 을 돌려준다. 표는 N = M = 352 에 고정이다
//   (software/golden_develop/region_check.py 가 쓴다).
//
// 병합 단계 (P)
//   이어 오는 이벤트는 같은 칸에 맞는 일이 잦다: 한 그룹의 이벤트는 같은 열에서 1 px 씩
//   떨어진 행에서 오므로, 회전 뒤에도 여럿이 같은 4 x 4 px 칸에 떨어지는 일이 많다. 대기
//   중인 칸과 같은 이벤트는 그 카운트 p_n 만 올린다. 칸은 이벤트마다 한 번씩이 아니라
//   +p_n 으로 한 번만 읽고 쓴다. n 을 포화 덧셈으로 더한 값은 포화 +1 을 n 번 한 값과
//   같다. 대기 중인 갱신은 다른 칸이 오거나, 입력에 빈 클럭이 있거나, p_n 이 가득 차면
//   (2^CW - 1, CW = 4 이면 15) P 를 떠난다(발행된다).
//   CW = 1 이면 카운트가 늘 수 없어 모든 이벤트가 따로 발행된다(병합 없음).
//
// Read-modify-write
//   발행 사이클 : P 의 뱅크를 읽는다.                       (P -> W)
//   W 사이클    : 읽은 데이터에 w_n 을 2^BITS - 1 에서 포화하도록 더해 다시 쓴다.
//   W 사이클에는 뱅크가 바쁘므로 P 는 W 가 쓰는 뱅크로 발행할 수 없다. 한 클럭 기다리며,
//   그 사이 새 칸이 들어오면 ev_ready 로 입력을 멈춘다. 이 기다림 덕분에 한 칸의 다음
//   읽기는 언제나 그 칸의 쓰기가 끝난 뒤에 오므로 포워딩 경로가 필요 없다. 병합 뒤에
//   잇따르는 갱신은 서로 다른 칸이고, 대개 한 열의 선을 따라 이웃하거나 몇 칸 떨어진
//   칸이라 서로 다른 뱅크에 떨어지므로 이런 멈춤은 드물다.
//
// clr 은 저장하는 모든 칸에 0 을 쓴다. 모든 뱅크를 동시에, 클럭마다 워드 하나씩 쓴다
// (BD 클럭, 그동안 clr_on 1). 읽기 포트(rd_en, rd_x, rd_y -> 한 클럭 뒤 rd_data)는
// 이벤트와 뱅크 포트를 함께 쓴다. 둘 다 파이프라인이 idle 일 때 쓰라는 것이다: clr 은
// 진행 중인 쓰기를 덮어쓰고, P 가 칸을 잡고 있는 동안 읽기 포트는 (rd_x, rd_y) 대신
// 그 칸을 읽는다. wl_top 은 지우는 동안 새 입력 워드를 막는다. 읽기 포트를 idle 일
// 때만 쓰는 것은 부르는 쪽이 맡는다.
module wl_accum #(
    parameter N    = 352,
    parameter M    = 352,
    parameter BITS = 13,
    parameter CW   = 4          // 병합 카운트 폭, 1 = 병합하지 않음. CW <= BITS
) (
    input  wire            clk,
    input  wire            rst_n,

    input  wire            ev_valid,
    output wire            ev_ready,
    input  wire      [8:0] ev_x,
    input  wire      [8:0] ev_y,

    input  wire            clr,
    output reg             clr_on,

    input  wire            rd_en,
    input  wire      [8:0] rd_x,
    input  wire      [8:0] rd_y,
    output wire [BITS-1:0] rd_data,

    output wire            busy
);
    localparam            BK   = 2;                          // 축마다 뱅크 번호 비트 수
    localparam            NB   = 16;                         // 뱅크 수
    localparam            BD   = 6132;                       // 뱅크마다 저장하는 칸 수(wl_region_map)
    localparam            AW   = 13;                         // 워드 주소 비트 수, 2^13 >= BD
    localparam [BITS-1:0] MAX  = {BITS{1'b1}};
    localparam   [CW-1:0] NMAX = {CW{1'b1}};

    // ---------------------------------------------------------------- P
    reg                   p_v;
    reg             [8:0] p_x, p_y;
    reg          [CW-1:0] p_n;
    wire       [2*BK-1:0] p_b = {p_y[BK-1:0], p_x[BK-1:0]};

    // ---------------------------------------------------------------- W
    reg                   w_v;
    reg        [2*BK-1:0] w_b;      // 마지막으로 읽은 뱅크(W, 또는 읽기 포트)
    reg          [AW-1:0] w_a;
    reg          [CW-1:0] w_n;

    // conflict: 이 클럭에 P 가 읽을 뱅크에 W 가 쓰므로 P 는 발행할 수 없다.
    // ev_ready 는 들어온 이벤트가 P 에 병합될 수 없고(다른 칸이거나 카운트가 가득 참)
    // P 가 발행해서 자리를 비울 수도 없을 때만 0 이다.
    wire                  conflict = w_v && (w_b == p_b);
    wire                  match    = p_v && (ev_x == p_x) && (ev_y == p_y) && (p_n != NMAX);
    wire                  take     = ev_valid && match;              // P 에 병합한다
    wire                  issue    = p_v && !take && !conflict;      // P 가 자기 뱅크를 읽는다
    assign ev_ready = !p_v || match || !conflict;
    wire                  load     = ev_valid && ev_ready && !match; // 새 칸을 P 에 넣는다

    // 읽기 주소: 이벤트가 흐르는 동안은 P, idle 일 때는 읽기 포트.
    wire            [8:0] qx = p_v ? p_x : rd_x;
    wire            [8:0] qy = p_v ? p_y : rd_y;
    wire       [2*BK-1:0] qb = {qy[BK-1:0], qx[BK-1:0]};
    wire         [AW-1:0] qa;
    wire                  q_hit;
    wire                  rq = issue || rd_en;
    reg                   rd_hit;   // 읽기 포트로 마지막에 읽은 칸이 저장되는 칸이다

    wl_region_map u_map (.wx(qx), .wy(qy), .hit(q_hit), .addr(qa));

    reg          [AW-1:0] clr_a;
    wire       [BITS-1:0] rdata [0:NB-1];
    wire       [BITS-1:0] old = rdata[w_b];
    wire         [BITS:0] sum = {1'b0, old} + w_n;
    wire       [BITS-1:0] nw  = sum[BITS] ? MAX : sum[BITS-1:0];

    assign rd_data = rd_hit ? old : {BITS{1'b0}};
    assign busy    = p_v || w_v || clr_on;

    genvar i;
    generate
        for (i = 0; i < NB; i = i + 1) begin : g_bank
            // 뱅크는 지우는 동안(모든 뱅크, 워드 clr_a, 데이터 0)과 자기 갱신의 W 사이클에
            // 쓴다. 그 밖에는 읽기 주소가 그 뱅크를 고를 때 읽는다(P 의 발행, 또는
            // 읽기 포트).
            wire wr = clr_on || (w_v && w_b == i);
            // 합성에서는 뱅크를 hardware/syn/wl_sram.lib 의 타이밍만 있는 셀에 묶는다.
            // 그 셀은 파라미터를 받을 수 없고, 핀은 기본값 BITS = 13 일 때의 핀이다.
`ifdef WL_SRAM_LIBCELL
            wl_sram u_mem (
`else
            wl_sram #(.AW(AW), .DW(BITS), .DEPTH(BD)) u_mem (
`endif
                .clk   (clk),
                .en    (wr || (rq && qb == i)),
                .we    (wr),
                .addr  (clr_on ? clr_a : wr ? w_a : qa),
                .wd    (clr_on ? {BITS{1'b0}} : nw),
                .rdata (rdata[i])
            );
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clr_on <= 1'b0;
            clr_a  <= {AW{1'b0}};
            p_v    <= 1'b0;
            p_x    <= 9'd0;
            p_y    <= 9'd0;
            p_n    <= {CW{1'b0}};
            w_v    <= 1'b0;
            w_b    <= {(2*BK){1'b0}};
            w_a    <= {AW{1'b0}};
            w_n    <= {CW{1'b0}};
            rd_hit <= 1'b0;
        end else begin
            if (take) begin
                p_n <= p_n + 1'b1;
            end else if (load) begin
                p_v <= 1'b1;
                p_x <= ev_x;
                p_y <= ev_y;
                p_n <= 1'b1;
            end else if (issue) begin
                p_v <= 1'b0;
            end

            w_v <= issue;
            if (rq) begin
                w_b <= qb;
                w_a <= qa;
                w_n <= p_n;
            end
            if (rd_en) rd_hit <= q_hit;

            if (clr && !clr_on) begin
                clr_on <= 1'b1;
                clr_a  <= {AW{1'b0}};
            end else if (clr_on) begin
                clr_a <= clr_a + 1'b1;
                if (clr_a == BD - 1) clr_on <= 1'b0;
            end
        end
    end
endmodule
