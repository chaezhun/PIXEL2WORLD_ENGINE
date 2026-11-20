// wl_region : camera roll 이 쓸고 지나가는 원만 저장할 때 월드 칸의 bank, macro,
// macro 주소. 출력은 레지스터로 한 클록 늦춰 낸다.
//
// 이 파일에는 모듈이 셋 있다. wl_region 과 wl_region_sq 는 비교용 모듈이다:
// 둘을 나란히 합성해 원 매핑의 로직 비용을 단순 정사각형 매핑과 견준다.
// 설계 자체는 wl_region_map(이 파일 맨 끝)만 인스턴스한다. 이는 wl_region 과
// 같은 원 표에서 레지스터와 macro 나눔을 뺀 것이다.
//
// 월드 메모리는 wl_accum 처럼 bank 16 개로 나뉜다: bank =
// {wy[1:0], wx[1:0]} 이고, bank 안에서 칸은 88 x 88 격자의 bank 행 r = wy >> 2,
// bank 열 c = wx >> 2 에 놓인다. 격자 전체를 저장하면 주소가
// r * 88 + c 이고 bank 당 7,744 word 가 든다(wl_region_sq).
//
// roll 에서는 모든 픽셀이 회전 중심 둘레를 돌고 회전 중심은 월드 중심에 놓이므로,
// 쓰기는 월드 중심 둘레의 원 안에서만 일어난다. 여기서 bank 는 반지름 696 px
// 원의 칸만 둔다: bank 행 r 은 열 START[r] .. END[r] - 1 을 담고 행을 차례로
// 이어 붙이므로, 주소는 BASE[r] + c - START[r] 이다. BASE[r] 은 행
// 0 .. r - 1 의 칸 수다. 표 하나를 bank 16 개가 같이 쓰고(16 개 원 칸의 합집합)
// bank 당 6132 word 가 필요하다. 이는 4,096-word macro 하나와 1,024-word
// macro 두 개(6,144 word)에 들어간다. 반지름은 스크립트가 시도한 것 가운데 표가 이
// macro 조합에 아직 들어가는 가장 큰 값이다. 28 비트 표 항목은 각각
// {OFF, START, END} 이다: OFF = BASE - START(14 비트 2의 보수)는 [27:14],
// START 는 [13:7], END 는 [6:0] 에 있다. 표 밖의 칸은 저장하지 않는다.
//
// software/golden_develop/region_check.py 가 자기 표로 이 파일을 쓴다. 이 파일이 아니라
// 스크립트를 고친다.
module wl_region (
    input  wire        clk,
    input  wire        in_valid,
    input  wire [8:0]  wx,          // 월드 칸 열, 0 .. 351
    input  wire [8:0]  wy,          // 월드 칸 행, 0 .. 351
    output reg         out_valid,   // 한 클록 뒤의 in_valid, 원 밖이면 0
    output reg  [3:0]  bank,
    output reg  [1:0]  macro,       // 0: 4,096-word macro, 1 과 2: 1,024-word macro
    output reg  [11:0] maddr        // 그 macro 안의 word
);
    function [27:0] rom(input [6:0] r);
        case (r)
            7'd0: rom = {14'h3fdb, 7'd37, 7'd51};
            7'd1: rom = {14'h3fee, 7'd32, 7'd56};
            7'd2: rom = {14'h0009, 7'd29, 7'd59};
            7'd3: rom = {14'h002a, 7'd26, 7'd62};
            7'd4: rom = {14'h0050, 7'd24, 7'd64};
            7'd5: rom = {14'h007a, 7'd22, 7'd66};
            7'd6: rom = {14'h00a7, 7'd21, 7'd67};
            7'd7: rom = {14'h00d7, 7'd19, 7'd69};
            7'd8: rom = {14'h010a, 7'd18, 7'd70};
            7'd9: rom = {14'h0140, 7'd16, 7'd72};
            7'd10: rom = {14'h0179, 7'd15, 7'd73};
            7'd11: rom = {14'h01b4, 7'd14, 7'd74};
            7'd12: rom = {14'h01f1, 7'd13, 7'd75};
            7'd13: rom = {14'h0230, 7'd12, 7'd76};
            7'd14: rom = {14'h0271, 7'd11, 7'd77};
            7'd15: rom = {14'h02b4, 7'd10, 7'd78};
            7'd16: rom = {14'h02f9, 7'd9, 7'd79};
            7'd17: rom = {14'h033f, 7'd9, 7'd79};
            7'd18: rom = {14'h0386, 7'd8, 7'd80};
            7'd19: rom = {14'h03cf, 7'd7, 7'd81};
            7'd20: rom = {14'h0419, 7'd7, 7'd81};
            7'd21: rom = {14'h0464, 7'd6, 7'd82};
            7'd22: rom = {14'h04b1, 7'd5, 7'd83};
            7'd23: rom = {14'h04ff, 7'd5, 7'd83};
            7'd24: rom = {14'h054e, 7'd4, 7'd84};
            7'd25: rom = {14'h059e, 7'd4, 7'd84};
            7'd26: rom = {14'h05ef, 7'd3, 7'd85};
            7'd27: rom = {14'h0641, 7'd3, 7'd85};
            7'd28: rom = {14'h0693, 7'd3, 7'd85};
            7'd29: rom = {14'h06e6, 7'd2, 7'd86};
            7'd30: rom = {14'h073a, 7'd2, 7'd86};
            7'd31: rom = {14'h078e, 7'd2, 7'd86};
            7'd32: rom = {14'h07e3, 7'd1, 7'd87};
            7'd33: rom = {14'h0839, 7'd1, 7'd87};
            7'd34: rom = {14'h088f, 7'd1, 7'd87};
            7'd35: rom = {14'h08e5, 7'd1, 7'd87};
            7'd36: rom = {14'h093b, 7'd1, 7'd87};
            7'd37: rom = {14'h0992, 7'd0, 7'd88};
            7'd38: rom = {14'h09ea, 7'd0, 7'd88};
            7'd39: rom = {14'h0a42, 7'd0, 7'd88};
            7'd40: rom = {14'h0a9a, 7'd0, 7'd88};
            7'd41: rom = {14'h0af2, 7'd0, 7'd88};
            7'd42: rom = {14'h0b4a, 7'd0, 7'd88};
            7'd43: rom = {14'h0ba2, 7'd0, 7'd88};
            7'd44: rom = {14'h0bfa, 7'd0, 7'd88};
            7'd45: rom = {14'h0c52, 7'd0, 7'd88};
            7'd46: rom = {14'h0caa, 7'd0, 7'd88};
            7'd47: rom = {14'h0d02, 7'd0, 7'd88};
            7'd48: rom = {14'h0d5a, 7'd0, 7'd88};
            7'd49: rom = {14'h0db2, 7'd0, 7'd88};
            7'd50: rom = {14'h0e0a, 7'd0, 7'd88};
            7'd51: rom = {14'h0e61, 7'd1, 7'd87};
            7'd52: rom = {14'h0eb7, 7'd1, 7'd87};
            7'd53: rom = {14'h0f0d, 7'd1, 7'd87};
            7'd54: rom = {14'h0f63, 7'd1, 7'd87};
            7'd55: rom = {14'h0fb9, 7'd1, 7'd87};
            7'd56: rom = {14'h100e, 7'd2, 7'd86};
            7'd57: rom = {14'h1062, 7'd2, 7'd86};
            7'd58: rom = {14'h10b6, 7'd2, 7'd86};
            7'd59: rom = {14'h1109, 7'd3, 7'd85};
            7'd60: rom = {14'h115b, 7'd3, 7'd85};
            7'd61: rom = {14'h11ad, 7'd3, 7'd85};
            7'd62: rom = {14'h11fe, 7'd4, 7'd84};
            7'd63: rom = {14'h124e, 7'd4, 7'd84};
            7'd64: rom = {14'h129d, 7'd5, 7'd83};
            7'd65: rom = {14'h12eb, 7'd5, 7'd83};
            7'd66: rom = {14'h1338, 7'd6, 7'd82};
            7'd67: rom = {14'h1383, 7'd7, 7'd81};
            7'd68: rom = {14'h13cd, 7'd7, 7'd81};
            7'd69: rom = {14'h1416, 7'd8, 7'd80};
            7'd70: rom = {14'h145d, 7'd9, 7'd79};
            7'd71: rom = {14'h14a3, 7'd9, 7'd79};
            7'd72: rom = {14'h14e8, 7'd10, 7'd78};
            7'd73: rom = {14'h152b, 7'd11, 7'd77};
            7'd74: rom = {14'h156c, 7'd12, 7'd76};
            7'd75: rom = {14'h15ab, 7'd13, 7'd75};
            7'd76: rom = {14'h15e8, 7'd14, 7'd74};
            7'd77: rom = {14'h1623, 7'd15, 7'd73};
            7'd78: rom = {14'h165c, 7'd16, 7'd72};
            7'd79: rom = {14'h1692, 7'd18, 7'd70};
            7'd80: rom = {14'h16c5, 7'd19, 7'd69};
            7'd81: rom = {14'h16f5, 7'd21, 7'd67};
            7'd82: rom = {14'h1722, 7'd22, 7'd66};
            7'd83: rom = {14'h174c, 7'd24, 7'd64};
            7'd84: rom = {14'h1772, 7'd26, 7'd62};
            7'd85: rom = {14'h1793, 7'd29, 7'd59};
            7'd86: rom = {14'h17ae, 7'd32, 7'd56};
            7'd87: rom = {14'h17c1, 7'd37, 7'd51};
            default: rom = 28'd0;   // 빈 행: START = END = 0, 안에 든 칸이 없다
        endcase
    endfunction

    wire [6:0]  r = wy[8:2];
    wire [6:0]  c = wx[8:2];
    wire [27:0] t = rom(r);
    wire        in_row = (c >= t[13:7]) && (c < t[6:0]);
    wire [13:0] a = t[27:14] + {7'd0, c};   // bank 안 word, in_row 이면 0 .. 6131

    // word 0 .. 4,095 는 macro 0, 4,096 .. 5,119 는 macro 1,
    // 5,120 .. 6,143 은 macro 2 로 간다(a[10] 이 작은 macro 둘 중 하나를 고른다).
    always @(posedge clk) begin
        out_valid <= in_valid & in_row;
        bank      <= {wy[1:0], wx[1:0]};
        macro     <= a[12] ? (a[10] ? 2'd2 : 2'd1) : 2'd0;
        maddr     <= a[12] ? {2'd0, a[9:0]} : a[11:0];
    end
endmodule

// wl_region_sq : 비교용 모듈. 표 없는 단순 정사각형 매핑이다:
// 주소 r * 88 + c(0 .. 7,743)를 bank 마다 4,096-word macro 두 개에 나눠 담으므로
// 버리는 칸이 없다(out_valid = in_valid). wl_region 과 포트가 같아 둘을
// 나란히 합성할 수 있다.
module wl_region_sq (
    input  wire        clk,
    input  wire        in_valid,
    input  wire [8:0]  wx,
    input  wire [8:0]  wy,
    output reg         out_valid,
    output reg  [3:0]  bank,
    output reg  [1:0]  macro,
    output reg  [11:0] maddr
);
    wire [12:0] a = {6'd0, wy[8:2]} * 13'd88 + {6'd0, wx[8:2]};   // 0 .. 7,743

    always @(posedge clk) begin
        out_valid <= in_valid;
        bank      <= {wy[1:0], wx[1:0]};
        macro     <= {1'b0, a[12]};
        maddr     <= a[11:0];
    end
endmodule

// wl_region_map : wl_region 의 표를 조합 논리로 둔 것으로, 레지스터와 macro 나눔이
// 없다. 설계가 쓰는 매핑이 이것이다: wl_xform 은 칸이 표 밖인 이벤트를 버리고
// (hit 0), wl_accum 은 addr 를 bank {wy[1:0], wx[1:0]} 안의 word 로 삼아 갱신과
// 읽기에 쓴다. bank 하나는 6132 word 를 담고, 이를 macro 로 어떻게 나눌지는
// 메모리 쪽에 맡긴다.
// 좌표 352 .. 511 은 r 이나 c 가 88 .. 127 이 되어 모든 표 행 밖이므로
// (행 88 .. 127 은 비어 있고 END <= 88) 거기서는 hit 가 0 이다.
module wl_region_map (
    input  wire [8:0]  wx,
    input  wire [8:0]  wy,
    output wire        hit,         // 이 칸을 저장한다
    output wire [12:0] addr         // 그 bank 안의 word, hit 이면 0 .. 6131
);
    function [27:0] rom(input [6:0] r);
        case (r)
            7'd0: rom = {14'h3fdb, 7'd37, 7'd51};
            7'd1: rom = {14'h3fee, 7'd32, 7'd56};
            7'd2: rom = {14'h0009, 7'd29, 7'd59};
            7'd3: rom = {14'h002a, 7'd26, 7'd62};
            7'd4: rom = {14'h0050, 7'd24, 7'd64};
            7'd5: rom = {14'h007a, 7'd22, 7'd66};
            7'd6: rom = {14'h00a7, 7'd21, 7'd67};
            7'd7: rom = {14'h00d7, 7'd19, 7'd69};
            7'd8: rom = {14'h010a, 7'd18, 7'd70};
            7'd9: rom = {14'h0140, 7'd16, 7'd72};
            7'd10: rom = {14'h0179, 7'd15, 7'd73};
            7'd11: rom = {14'h01b4, 7'd14, 7'd74};
            7'd12: rom = {14'h01f1, 7'd13, 7'd75};
            7'd13: rom = {14'h0230, 7'd12, 7'd76};
            7'd14: rom = {14'h0271, 7'd11, 7'd77};
            7'd15: rom = {14'h02b4, 7'd10, 7'd78};
            7'd16: rom = {14'h02f9, 7'd9, 7'd79};
            7'd17: rom = {14'h033f, 7'd9, 7'd79};
            7'd18: rom = {14'h0386, 7'd8, 7'd80};
            7'd19: rom = {14'h03cf, 7'd7, 7'd81};
            7'd20: rom = {14'h0419, 7'd7, 7'd81};
            7'd21: rom = {14'h0464, 7'd6, 7'd82};
            7'd22: rom = {14'h04b1, 7'd5, 7'd83};
            7'd23: rom = {14'h04ff, 7'd5, 7'd83};
            7'd24: rom = {14'h054e, 7'd4, 7'd84};
            7'd25: rom = {14'h059e, 7'd4, 7'd84};
            7'd26: rom = {14'h05ef, 7'd3, 7'd85};
            7'd27: rom = {14'h0641, 7'd3, 7'd85};
            7'd28: rom = {14'h0693, 7'd3, 7'd85};
            7'd29: rom = {14'h06e6, 7'd2, 7'd86};
            7'd30: rom = {14'h073a, 7'd2, 7'd86};
            7'd31: rom = {14'h078e, 7'd2, 7'd86};
            7'd32: rom = {14'h07e3, 7'd1, 7'd87};
            7'd33: rom = {14'h0839, 7'd1, 7'd87};
            7'd34: rom = {14'h088f, 7'd1, 7'd87};
            7'd35: rom = {14'h08e5, 7'd1, 7'd87};
            7'd36: rom = {14'h093b, 7'd1, 7'd87};
            7'd37: rom = {14'h0992, 7'd0, 7'd88};
            7'd38: rom = {14'h09ea, 7'd0, 7'd88};
            7'd39: rom = {14'h0a42, 7'd0, 7'd88};
            7'd40: rom = {14'h0a9a, 7'd0, 7'd88};
            7'd41: rom = {14'h0af2, 7'd0, 7'd88};
            7'd42: rom = {14'h0b4a, 7'd0, 7'd88};
            7'd43: rom = {14'h0ba2, 7'd0, 7'd88};
            7'd44: rom = {14'h0bfa, 7'd0, 7'd88};
            7'd45: rom = {14'h0c52, 7'd0, 7'd88};
            7'd46: rom = {14'h0caa, 7'd0, 7'd88};
            7'd47: rom = {14'h0d02, 7'd0, 7'd88};
            7'd48: rom = {14'h0d5a, 7'd0, 7'd88};
            7'd49: rom = {14'h0db2, 7'd0, 7'd88};
            7'd50: rom = {14'h0e0a, 7'd0, 7'd88};
            7'd51: rom = {14'h0e61, 7'd1, 7'd87};
            7'd52: rom = {14'h0eb7, 7'd1, 7'd87};
            7'd53: rom = {14'h0f0d, 7'd1, 7'd87};
            7'd54: rom = {14'h0f63, 7'd1, 7'd87};
            7'd55: rom = {14'h0fb9, 7'd1, 7'd87};
            7'd56: rom = {14'h100e, 7'd2, 7'd86};
            7'd57: rom = {14'h1062, 7'd2, 7'd86};
            7'd58: rom = {14'h10b6, 7'd2, 7'd86};
            7'd59: rom = {14'h1109, 7'd3, 7'd85};
            7'd60: rom = {14'h115b, 7'd3, 7'd85};
            7'd61: rom = {14'h11ad, 7'd3, 7'd85};
            7'd62: rom = {14'h11fe, 7'd4, 7'd84};
            7'd63: rom = {14'h124e, 7'd4, 7'd84};
            7'd64: rom = {14'h129d, 7'd5, 7'd83};
            7'd65: rom = {14'h12eb, 7'd5, 7'd83};
            7'd66: rom = {14'h1338, 7'd6, 7'd82};
            7'd67: rom = {14'h1383, 7'd7, 7'd81};
            7'd68: rom = {14'h13cd, 7'd7, 7'd81};
            7'd69: rom = {14'h1416, 7'd8, 7'd80};
            7'd70: rom = {14'h145d, 7'd9, 7'd79};
            7'd71: rom = {14'h14a3, 7'd9, 7'd79};
            7'd72: rom = {14'h14e8, 7'd10, 7'd78};
            7'd73: rom = {14'h152b, 7'd11, 7'd77};
            7'd74: rom = {14'h156c, 7'd12, 7'd76};
            7'd75: rom = {14'h15ab, 7'd13, 7'd75};
            7'd76: rom = {14'h15e8, 7'd14, 7'd74};
            7'd77: rom = {14'h1623, 7'd15, 7'd73};
            7'd78: rom = {14'h165c, 7'd16, 7'd72};
            7'd79: rom = {14'h1692, 7'd18, 7'd70};
            7'd80: rom = {14'h16c5, 7'd19, 7'd69};
            7'd81: rom = {14'h16f5, 7'd21, 7'd67};
            7'd82: rom = {14'h1722, 7'd22, 7'd66};
            7'd83: rom = {14'h174c, 7'd24, 7'd64};
            7'd84: rom = {14'h1772, 7'd26, 7'd62};
            7'd85: rom = {14'h1793, 7'd29, 7'd59};
            7'd86: rom = {14'h17ae, 7'd32, 7'd56};
            7'd87: rom = {14'h17c1, 7'd37, 7'd51};
            default: rom = 28'd0;
        endcase
    endfunction

    wire [6:0]  r = wy[8:2];
    wire [6:0]  c = wx[8:2];
    wire [27:0] t = rom(r);
    wire [13:0] a = t[27:14] + {7'd0, c};

    assign hit  = (c >= t[13:7]) && (c < t[6:0]);
    assign addr = a[12:0];
endmodule
