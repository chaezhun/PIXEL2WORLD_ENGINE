// tb_base : 기준 설계 wl_base_top 에 이벤트 단위 입력을 넣고 월드 메모리를 골든 모델과 비교한다.
// wl_base_top 은 wl_top 의 MGROUP 워드 흐름 대신 입력 하나에 이벤트 (x, y, phase) 하나를
// 받는다.
//
// 벡터 디렉터리는 +vec=<dir> 로 준다.
//   events.hex    한 줄에 이벤트 하나, {phase[15:0], y[9:0], x[9:0]} 를 hex 9자리로 쓴다.
//                 위상은 그 이벤트가 든 프레임의 위상이다
//                 (golden.py events <recording> <bundle> <f0> <f1> <dir>)
//   expected.hex  누적한 뒤의 월드 메모리, 주소 0 .. N*M-1. 같은 프레임의 wl_top 벡터와
//                 같은 파일이다
// 선택 plusarg: +cx=<px> +cy=<px> (기본 533 418), +vcd=<file> 과 +vcd_from=<n> +vcd_len=<n>
// 은 tb_worldlock 과 같다(Verilator 는 --trace 가 필요하다).
//
// 입력은 매 클럭 내준다. 모든 칸이 일치하면 "RESULT PASS" 를 출력한다.
`timescale 1ns/1ps
module tb_base;
    parameter N    = 352;
    parameter M    = 352;
    parameter BITS = 13;
    parameter MAXE = 1 << 22;            // 이벤트 버퍼
    localparam DEPTH = N * M;

    reg clk = 1'b0;
    always #2.5 clk = ~clk;              // 200 MHz. 사이클 수만 의미가 있다

    reg  [35:0]     evs  [0:MAXE-1];
    reg  [BITS-1:0] expv [0:DEPTH-1];
    integer n_evs, n_exp;

    reg             rst_n = 1'b0;
    reg       [9:0] cx, cy;
    reg             clr = 1'b0;
    reg             rd_en = 1'b0;
    reg      [16:0] rd_addr = 17'd0;
    reg             feeding = 1'b0;
    integer         ei = 0;

    wire            in_ready, clr_busy, idle;
    wire [BITS-1:0] rd_data;
    wire     [23:0] drop_cnt;
    wire            in_valid = feeding && (ei < n_evs);
    wire     [35:0] ev = evs[ei];

    wl_base_top #(.N(N), .M(M), .BITS(BITS)) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .cx       (cx),
        .cy       (cy),
        .in_x     (ev[9:0]),
        .in_y     (ev[19:10]),
        .in_phase (ev[35:20]),
        .in_valid (in_valid),
        .in_ready (in_ready),
        .clr      (clr),
        .clr_busy (clr_busy),
        .rd_en    (rd_en),
        .rd_x     (rd_addr % N),
        .rd_y     (rd_addr / N),
        .rd_data  (rd_data),
        .idle     (idle),
        .drop_cnt (drop_cnt)
    );

    integer cycle = 0;
    always @(posedge clk) begin
        cycle <= cycle + 1;
        if (in_valid && in_ready) ei <= ei + 1;
        if (cycle > 8 * MAXE + 4 * DEPTH + 100000) begin
            $display("TIMEOUT at cycle %0d: event %0d / %0d, idle %0d", cycle, ei, n_evs, idle);
            $display("RESULT FAIL");
            $finish;
        end
    end

    // tb_worldlock 과 같은 카운터: 병합된 갱신과 받지 않은 이벤트.
    integer n_ev = 0, n_upd = 0, n_stall = 0;
    always @(posedge clk) if (feeding) begin
        if (dut.ev_valid && dut.ev_ready)  n_ev    <= n_ev + 1;
        if (dut.ev_valid && !dut.ev_ready) n_stall <= n_stall + 1;
        if (dut.u_acc.issue)               n_upd   <= n_upd + 1;
    end

    reg [8*512-1:0] vcd_file;
    integer vcd_from = 0, vcd_len = 20000, vcd_ev0 = 0;
    initial begin
        if ($value$plusargs("vcd=%s", vcd_file)) begin
            if (!$value$plusargs("vcd_from=%d", vcd_from)) vcd_from = 0;
            if (!$value$plusargs("vcd_len=%d", vcd_len)) vcd_len = 20000;
            wait (feeding);
            repeat (vcd_from) @(posedge clk);
            vcd_ev0 = n_ev;
            $dumpfile(vcd_file);
            $dumpvars(0, dut);
            repeat (vcd_len) @(posedge clk);
            $dumpoff;
            $dumpflush;
            $display("vcd %0d clocks from clock %0d after start, %0d events in the window",
                     vcd_len, vcd_from, n_ev - vcd_ev0);
        end
    end

    reg             rd_v = 1'b0;
    reg      [16:0] rd_a1 = 17'd0;
    integer         n_bad = 0, n_chk = 0, sum = 0;
    always @(posedge clk) begin
        rd_v  <= rd_en;
        rd_a1 <= rd_addr;
        if (rd_v) begin
            n_chk <= n_chk + 1;
            sum   <= sum + rd_data;
            if (rd_data !== expv[rd_a1]) begin
                if (n_bad < 10)
                    $display("MISMATCH addr %0d (wx %0d wy %0d): got %0d expected %0d",
                             rd_a1, rd_a1 % N, rd_a1 / N, rd_data, expv[rd_a1]);
                n_bad <= n_bad + 1;
            end
        end
    end

    reg [8*512-1:0] vec, path;
    integer a, t_start, t_end, tmp, fd, r;
    reg [35:0] w;
    initial begin
        if (!$value$plusargs("vec=%s", vec)) vec = "vec";
        if (!$value$plusargs("cx=%d", tmp)) tmp = 533;
        cx = tmp;
        if (!$value$plusargs("cy=%d", tmp)) tmp = 418;
        cy = tmp;
        $sformat(path, "%0s/events.hex", vec);
        fd = $fopen(path, "r");
        if (fd == 0) begin $display("cannot open %0s", path); $finish; end
        n_evs = 0;
        r = $fscanf(fd, "%h\n", w);
        while (r == 1 && n_evs < MAXE) begin
            evs[n_evs] = w;
            n_evs = n_evs + 1;
            r = $fscanf(fd, "%h\n", w);
        end
        $fclose(fd);
        $sformat(path, "%0s/expected.hex", vec);
        fd = $fopen(path, "r");
        if (fd == 0) begin $display("cannot open %0s", path); $finish; end
        n_exp = 0;
        r = $fscanf(fd, "%h\n", w);
        while (r == 1 && n_exp < DEPTH) begin
            // tb_worldlock 과 같이 2^BITS - 1 로 자른다
            expv[n_exp] = (w > {BITS{1'b1}}) ? {BITS{1'b1}} : w[BITS-1:0];
            n_exp = n_exp + 1;
            r = $fscanf(fd, "%h\n", w);
        end
        $fclose(fd);
        $display("loaded %0d events, %0d cells; centre (%0d, %0d)", n_evs, n_exp, cx, cy);

        repeat (4) @(posedge clk);
        rst_n <= 1'b1;
        @(posedge clk);
        clr <= 1'b1;
        @(posedge clk);
        clr <= 1'b0;
        @(posedge clk);
        while (clr_busy) @(posedge clk);

        feeding <= 1'b1;
        t_start = cycle;
        @(posedge clk);
        while (ei < n_evs || !idle) @(posedge clk);
        t_end = cycle;
        feeding <= 1'b0;

        for (a = 0; a < DEPTH; a = a + 1) begin
            rd_en   <= 1'b1;
            rd_addr <= a;
            @(posedge clk);
        end
        rd_en <= 1'b0;
        repeat (3) @(posedge clk);

        $display("cycles %0d for %0d events; dropped %0d", t_end - t_start, n_evs, drop_cnt);
        $display("sram %0d updates for %0d events (%0.3f per event), %0d accesses, %0d stall cycles",
                 n_upd, n_ev, n_upd * 1.0 / (n_ev > 0 ? n_ev : 1), 2 * n_upd, n_stall);
        $display("memory sum %0d, checked %0d cells, mismatches %0d", sum, n_chk, n_bad);
        if (n_bad == 0 && n_chk == DEPTH && n_exp == DEPTH) $display("RESULT PASS");
        else $display("RESULT FAIL");
        $finish;
    end
endmodule
