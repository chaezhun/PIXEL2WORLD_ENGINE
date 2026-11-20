// 대회 서버 합성 흐름을 점검하는 최소 모듈.
//
// 입력 두 개를 레지스터에 받은 뒤 더해서 다시 레지스터에 담는다. 레지스터에서 레지스터로 가는
// 경로(a_q, b_q -> sum)가 하나 생기므로 합성 리포트에 면적과 타이밍이 함께 나온다.
// 설계 본체와는 관계가 없고, 라이브러리 · 제약 · 스크립트가 맞물리는지만 본다.
module smoke_top (
    input  wire       clk,
    input  wire       rst_n,   // 비동기 리셋, 0 일 때 리셋
    input  wire [7:0] a,
    input  wire [7:0] b,
    output reg  [8:0] sum      // 올림수까지 담으려고 9비트
);
    reg [7:0] a_q, b_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_q <= 8'd0;
            b_q <= 8'd0;
            sum <= 9'd0;
        end else begin
            a_q <= a;
            b_q <= b;
            sum <= a_q + b_q;
        end
    end
endmodule
