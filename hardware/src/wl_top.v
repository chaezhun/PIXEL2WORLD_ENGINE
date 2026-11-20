// wl_top : 이벤트를 월드 메모리에 쌓는 누산기의 최상위 모듈.
//
// 회전하는 이벤트 카메라의 32비트 원시 워드열(MGROUP 형식: 열 주소 뒤에 8행 비트맵을
// 하나나 둘씩 담은 이벤트 워드가 이어진다)을 받아, 각 이벤트를 그 프레임의 각도만큼
// 회전 중심 (cx, cy) 둘레로 돌리고, 떨어진 월드 메모리 칸에서 센다. 월드 메모리는
// 4 x 4 px 칸으로 된 N x M 격자이고, 그 중심(WC px)에 회전 중심이 온다. 실제로
// 저장하는 칸은 월드 중심 둘레의 원 안에 있는 칸뿐이다.
//
// 포트
//   in_word / in_valid / in_ready   MGROUP 원시 워드, valid/ready 핸드셰이크. 메모리를
//                                   지우는 동안은 막는다(in_ready 0).
//   ph_data / ph_valid / ph_ready   프레임마다 16비트 위상 하나, 프레임 순서대로
//                                   (2^16 = 360 deg). 실제 시스템에서는 그 프레임의
//                                   자이로 각도다. 위상 하나를 미리 버퍼에 받아 두므로
//                                   프레임보다 먼저 줘도 된다.
//   cx, cy                          회전 중심(px), 동작 중에는 바뀌지 않는다
//   clr / clr_busy                  저장하는 모든 칸을 0 으로 만든다. 그 동안 BD (= 6132)
//                                   클럭 clr_busy 가 1 이다. clr 은 idle 일 때만 올린다:
//                                   이미 처리 중인 이벤트를 붙잡아 두지 않는다.
//   rd_en / rd_x, rd_y / rd_data    칸 (wx, wy) 를 읽는다. 데이터는 한 클럭 뒤에 나온다.
//                                   저장하지 않는 칸은 0 으로 읽힌다. 읽기가 이벤트 경로와
//                                   뱅크 포트를 함께 쓰므로 idle 일 때만 쓴다
//                                   (여기서 강제하지는 않는다).
//   idle                            처리 중인 워드 · 작업 · 이벤트가 없다
//   frame_cnt                       리셋 뒤 본 프레임 경계 수
//   drop_cnt                        칸이 저장되지 않아(N x M 격자 밖이거나 원 밖)
//                                   세지 않은 이벤트 수
//
// 데이터 경로
//   wl_decode : MGROUP 워드 -> 작업(열 바뀜, 8행 그룹, 프레임 경계)
//   wl_xform  : 작업 -> 클럭마다 월드 칸 (wx, wy) 하나. 안에 있는 wl_cordic 이
//               프레임마다 위상을 (cos, sin) 으로 바꾼다
//   wl_accum  : 한 칸에 거듭 오는 이벤트를 합치고, 단일 포트 wl_sram 뱅크 16개에
//               read-modify-write 를 한다. 지우기와 읽기 포트도 맡는다
//
// 파라미터
//   N, M  월드 메모리 열 수와 행 수(칸). wl_region_map 의 저장 영역 표는 N = M = 352,
//         WC = 704 에 맞춰 만들었으므로 이 셋은 함께 정해진다.
//   BITS  칸당 비트 수, 포화하는 이벤트 카운터(13 -> 8191 에서 멈춘다)
//   CW    wl_accum 의 병합 카운트 폭, 1 = 병합하지 않음
//   WC    월드 중심(px), N * 4 / 2 (1408 px 정사각형의 가운데)
module wl_top #(
    parameter N    = 352,
    parameter M    = 352,
    parameter BITS = 13,
    parameter CW   = 4,         // wl_accum 의 병합 카운트 폭, 1 = 병합하지 않음
    parameter WC   = 704
) (
    input  wire            clk,
    input  wire            rst_n,

    input  wire      [9:0] cx,
    input  wire      [9:0] cy,

    input  wire     [31:0] in_word,
    input  wire            in_valid,
    output wire            in_ready,

    input  wire     [15:0] ph_data,
    input  wire            ph_valid,
    output wire            ph_ready,

    input  wire            clr,
    output wire            clr_busy,

    input  wire            rd_en,
    input  wire      [8:0] rd_x,
    input  wire      [8:0] rd_y,
    output wire [BITS-1:0] rd_data,

    output wire            idle,
    output wire     [15:0] frame_cnt,
    output wire     [23:0] drop_cnt
);
    wire        dec_ready, dec_busy;
    wire        j_valid, j_ready;
    wire  [1:0] j_type;
    wire  [9:0] j_val;
    wire  [7:0] j_map;
    wire        ev_valid, ev_ready;
    wire  [8:0] ev_x, ev_y;
    wire        xf_busy, acc_busy;

    // 지우는 동안에는 새 워드를 받지 않는다(디코더에도 in_valid 가 0 으로 보인다).
    assign in_ready = dec_ready && !clr_busy;
    assign idle     = !dec_busy && !xf_busy && !acc_busy;

    wl_decode u_dec (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_word   (in_word),
        .in_valid  (in_valid && !clr_busy),
        .in_ready  (dec_ready),
        .j_valid   (j_valid),
        .j_ready   (j_ready),
        .j_type    (j_type),
        .j_val     (j_val),
        .j_map     (j_map),
        .busy      (dec_busy),
        .frame_cnt (frame_cnt)
    );

    wl_xform #(.N(N), .M(M), .WC(WC)) u_xf (
        .clk      (clk),
        .rst_n    (rst_n),
        .cx       (cx),
        .cy       (cy),
        .ph_data  (ph_data),
        .ph_valid (ph_valid),
        .ph_ready (ph_ready),
        .j_valid  (j_valid),
        .j_ready  (j_ready),
        .j_type   (j_type),
        .j_val    (j_val),
        .j_map    (j_map),
        .ev_valid (ev_valid),
        .ev_ready (ev_ready),
        .ev_x     (ev_x),
        .ev_y     (ev_y),
        .drop_cnt (drop_cnt),
        .busy     (xf_busy)
    );

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
