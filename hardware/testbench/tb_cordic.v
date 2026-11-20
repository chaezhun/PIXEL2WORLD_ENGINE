// tb_cordic : 위상 65,536 개 모두에 대해 wl_cordic 을 돌리고 골든 표와 비교한다.
//
// +vec=<dir> 에는 cordic_all.hex 가 있어야 한다. p 번째 줄은 "CCCC SSSS" 로, 위상 p 의 cos 와
// sin 을 16비트 2의 보수 hex 로 쓴 것이며 software/golden_develop/golden.py 가 쓴다.
// 위상마다 따로 시작하고, done 이 1 이 되면 출력을 비교한다.
// 모든 위상이 일치하면 "RESULT PASS" 를 출력한다.
`timescale 1ns/1ps
module tb_cordic;
    reg clk = 1'b0;
    always #2.5 clk = ~clk;

    reg  [31:0] gold [0:65535];
    reg         rst_n = 1'b0;
    reg         start = 1'b0;
    reg  [15:0] phase = 16'd0;
    wire        busy, done;
    wire signed [15:0] c, s;

    wl_cordic dut (
        .clk   (clk),
        .rst_n (rst_n),
        .start (start),
        .phase (phase),
        .busy  (busy),
        .done  (done),
        .cos_o (c),
        .sin_o (s)
    );

    reg [8*512-1:0] vec, path;
    integer fd, r, p, n_bad;
    reg [15:0] gc, gs;
    initial begin
        if (!$value$plusargs("vec=%s", vec)) vec = "vec";
        $sformat(path, "%0s/cordic_all.hex", vec);
        fd = $fopen(path, "r");
        if (fd == 0) begin
            $display("cannot open %0s", path);
            $finish;
        end
        for (p = 0; p < 65536; p = p + 1) begin
            r = $fscanf(fd, "%h %h\n", gc, gs);
            gold[p] = {gc, gs};
        end
        $fclose(fd);

        n_bad = 0;
        repeat (3) @(posedge clk);
        rst_n <= 1'b1;
        @(posedge clk);
        for (p = 0; p < 65536; p = p + 1) begin
            phase <= p;
            start <= 1'b1;
            @(posedge clk);
            start <= 1'b0;
            @(posedge clk);
            while (!done) @(posedge clk);
            if ({c, s} !== gold[p]) begin
                if (n_bad < 10)
                    $display("MISMATCH phase %0d: got %0d %0d expected %0d %0d", p, c, s,
                             $signed(gold[p][31:16]), $signed(gold[p][15:0]));
                n_bad = n_bad + 1;
            end
        end
        $display("phases 65536, mismatches %0d", n_bad);
        if (n_bad == 0) $display("RESULT PASS");
        else $display("RESULT FAIL");
        $finish;
    end
endmodule
