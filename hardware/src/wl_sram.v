// wl_sram : 월드 메모리 뱅크 하나의 동작 모델(단일 포트, 1RW).
//
// 이 대회에는 GPDK045 용 SRAM 컴파일러가 없어서, 시뮬레이션에서는 이 모듈이
// 매크로를 대신한다. 합성은 이 파일을 읽지 않는다: wl_accum 을 WL_SRAM_LIBCELL 을
// 정의한 채로 읽어, 각 뱅크를 hardware/syn/wl_sram.lib 의 타이밍만 있는 셀
// wl_sram(면적 · 전력 0)에 묶는다. SRAM 의 면적과 전력은 가정한 매크로 수치로
// 따로 추정한다. 이 모듈을 플립플롭으로 합성해서는 안 된다.
// 컴파일한 매크로 하나보다 큰 뱅크는 이 핀들을 함께 쓰는 매크로 여러 개로 만든다
// (기본 6132 워드면 4,096 워드 하나와 1,024 워드 둘). 이 분할은 물리 구현의
// 세부라 여기서는 보이지 않는다.
//
// 클럭마다 접근은 한 번이다: en && we 이면 wd 를 addr 에 쓰고, en && !we 이면 addr 을
// 읽어 한 클럭 뒤 rdata 에 데이터가 나오며 다음 읽기까지 유지된다. 단일 포트라서
// 한 뱅크를 같은 클럭에 읽고 쓸 수 없고, wl_accum 은 그런 요청을 하지 않는다.
// 리셋 뒤 내용은 정해져 있지 않으며, wl_accum 이 먼저 지운다.
module wl_sram #(
    parameter AW    = 13,       // 주소 비트 수
    parameter DW    = 13,       // 워드 비트 수(월드 칸 카운터 하나)
    parameter DEPTH = 6132      // 뱅크의 워드 수
) (
    input  wire          clk,
    input  wire          en,
    input  wire          we,
    input  wire [AW-1:0] addr,
    input  wire [DW-1:0] wd,
    output reg  [DW-1:0] rdata
);
    reg [DW-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (en) begin
            if (we) mem[addr] <= wd;
            else    rdata     <= mem[addr];
        end
    end
endmodule
