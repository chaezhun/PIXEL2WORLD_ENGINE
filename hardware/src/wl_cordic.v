// wl_cordic : 16비트 phase -> (cos, sin), rotation mode 반복형 CORDIC.
//
// software/golden_develop/golden.py 의 cordic() 과 비트 단위로 같다.
//
//   phase  : 2^16 = 360 deg. [15:14] 는 사분면 q, [13:0] 은 그 안의 각 r 이다.
//   start  : z = (r << 3) - 65536. 곧 guard bit 3 개를 두고(z 의 단위는 2^-19 turn)
//            각을 [-45, 45) deg 로 옮긴다. 65536 * 2^-19 turn = 45 deg 이기 때문이다.
//            x = y = X0: 길이가 2^17 / K 인 45 deg 방향 벡터다. 여기서
//            K = i = 0..15 에 걸친 sqrt(1 + 2^-2i) 의 곱(약 1.6468)은 CORDIC
//            이득이다. X0 = round(2^17 / K / sqrt(2)) = 56281. 16 번 반복하고 나면
//            벡터는 각 r 을 가리키고 길이는 2^17 이다(소수 14 비트에
//            guard bit 3 개를 더한 1.0).
//   iter i : d = (z >= 0) ? +1 : -1
//            x <- x - d*(y >>> i), y <- y + d*(x >>> i), z <- z - d*ATAN[i]   (x, y 는 함께 갱신)
//   finish : x, y <- (v + 4) >>> 3, 그다음 q * 90 deg 만큼 되돌려 돌린다:
//            q = 0 1 2 3 -> (C, S) = (x, y) (-y, x) (-x, -y) (y, -x)
//
// 출력 C, S 는 소수 14 비트를 가진 부호 있는 16비트다(1.0 = 16384).
// 모든 phase 에서 x, y 는 부호 있는 19 비트, z 는 부호 있는 18 비트에 들어간다
// (hardware/testbench/tb_cordic.v 의 전수 시험으로 확인했다).
//
// 타이밍: start 는 idle 일 때 받는다. done 은 18 사이클 뒤에 한 클록 1 이 되고, 그때
// cos_o / sin_o 가 유효하다(다음 done 까지 유지된다).
module wl_cordic (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire        [15:0] phase,
    output wire               busy,
    output reg                done,
    output reg  signed [15:0] cos_o,
    output reg  signed [15:0] sin_o
);
    localparam signed [18:0] X0 = 19'sd56281;

    reg               run;      // 반복 중
    reg               fin;      // 마지막 반복 다음 한 사이클: 반올림하고 사분면을 되돌린다
    reg         [3:0] it;
    reg         [1:0] q;
    reg signed [18:0] x, y;
    reg signed [17:0] z;

    assign busy = run | fin;

    // ATAN[i] = round(atan(2^-i) / 2pi * 2^19): guard bit 3 개를 붙인 phase LSB 단위의 각.
    // ATAN[0] = 65536 이 45 deg 다. 반복 카운터 it 로 고른다.
    reg [16:0] atan;
    always @(*) begin
        case (it)
            4'd0:    atan = 17'd65536;
            4'd1:    atan = 17'd38688;
            4'd2:    atan = 17'd20442;
            4'd3:    atan = 17'd10377;
            4'd4:    atan = 17'd5208;
            4'd5:    atan = 17'd2607;
            4'd6:    atan = 17'd1304;
            4'd7:    atan = 17'd652;
            4'd8:    atan = 17'd326;
            4'd9:    atan = 17'd163;
            4'd10:   atan = 17'd81;
            4'd11:   atan = 17'd41;
            4'd12:   atan = 17'd20;
            4'd13:   atan = 17'd10;
            4'd14:   atan = 17'd5;
            default: atan = 17'd3;
        endcase
    end

    wire signed [17:0] at  = $signed({1'b0, atan});
    wire signed [18:0] xsh = x >>> it;
    wire signed [18:0] ysh = y >>> it;
    wire               dp  = ~z[17];                    // z >= 0

    // guard bit 3 개를 반올림한다. 결과는 16 비트에 들어간다(|v| <= 16386).
    wire signed [18:0] xr  = (x + 19'sd4) >>> 3;
    wire signed [18:0] yr  = (y + 19'sd4) >>> 3;
    wire signed [15:0] xo  = xr[15:0];
    wire signed [15:0] yo  = yr[15:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run   <= 1'b0;
            fin   <= 1'b0;
            done  <= 1'b0;
            it    <= 4'd0;
            q     <= 2'd0;
            x     <= 19'sd0;
            y     <= 19'sd0;
            z     <= 18'sd0;
            cos_o <= 16'sd0;
            sin_o <= 16'sd0;
        end else begin
            done <= 1'b0;
            fin  <= 1'b0;
            if (start && !busy) begin
                run <= 1'b1;
                it  <= 4'd0;
                q   <= phase[15:14];
                z   <= $signed({1'b0, phase[13:0], 3'b000}) - 18'sd65536;
                x   <= X0;
                y   <= X0;
            end else if (run) begin
                if (dp) begin
                    x <= x - ysh;
                    y <= y + xsh;
                    z <= z - at;
                end else begin
                    x <= x + ysh;
                    y <= y - xsh;
                    z <= z + at;
                end
                it <= it + 4'd1;
                if (it == 4'd15) begin
                    run <= 1'b0;
                    fin <= 1'b1;
                end
            end
            if (fin) begin
                done <= 1'b1;
                case (q)
                    2'd0: begin cos_o <=  xo; sin_o <=  yo; end
                    2'd1: begin cos_o <= -yo; sin_o <=  xo; end
                    2'd2: begin cos_o <= -xo; sin_o <= -yo; end
                    default: begin cos_o <= yo; sin_o <= -xo; end
                endcase
            end
        end
    end
endmodule
