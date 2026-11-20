// tb_worldlock : 골든 모델 벡터 묶음을 wl_top 에 넣고 월드 메모리를 비교한다.
//
// 벡터 파일은 software/golden_develop/golden.py 가 프레임 f0 .. f1-1 에 대해 쓰고, +vec=<dir> 로
// 준 디렉터리에서 읽는다. 모두 평문이고 한 줄에 hex 값 하나다.
//   words.hex     32비트 MGROUP 원본 워드(hex 8자리). 프레임 f0-1 을 닫는 프레임 끝 표지부터
//                 프레임 f1-1 을 닫는 표지까지다. f0 = 0 이면 파일은 녹화의 첫 워드부터
//                 시작한다.
//   phase.hex     프레임마다 16비트 위상 하나(hex 4자리). k 번째 프레임의 이벤트는
//                 k 번째 위상을 쓴다
//   expected.hex  빈 메모리에 이 프레임들을 누적한 뒤의 월드 메모리. 한 줄에 칸 하나씩
//                 주소 wy * N + wx, 0 .. N*M-1 순서다. N x M 정사각형 전체를 담으므로
//                 설계가 저장하지 않는 칸은 여기서 0 이다.
// 선택 plusarg: +cx=<px> +cy=<px> (기본 533 418), +dump=<file> 은 설계에서 다시 읽은 메모리를
// 쓰고, +gap=<n> 은 워드마다 뒤에 빈 사이클 n 개를 넣고, +trace 는 프레임 경계가 디코더를
// 떠나는 클럭을 출력하고, +vcd=<file> 은 활동 덤프를 쓴다(아래 VCD 블록 참고).
//
// 그 밖에는 입력을 매 클럭 내주므로, 출력되는 사이클 수는 코어 자체의 처리 시간이다.
// 모든 칸이 일치하고 끝난 프레임 수가 위상 수와 같으면 "RESULT PASS" 를 출력한다.
//
// +define+GLS 를 주면 설계는 합성된 넷리스트다(게이트 레벨 시뮬레이션,
// hardware/sim/run_gls.sh 가 실행한다). 넷리스트 top 에는 파라미터가 없고 내부 넷 이름이
// 바뀌므로, 파라미터 지정, SRAM 트래픽 카운터, 타임아웃 메시지의 내부 신호 관찰은 뺀다.
`timescale 1ns/1ps
module tb_worldlock;
    parameter N    = 352;
    parameter M    = 352;
    parameter BITS = 13;
    parameter CW   = 4;
    parameter MAXW = 1 << 20;            // 워드 버퍼. 녹화 전체(~28 M 워드)는 -GMAXW 가 필요하다
    parameter MAXF = 4096;               // 위상 버퍼
    localparam DEPTH = N * M;

    reg clk = 1'b0;
    always #2.5 clk = ~clk;              // 200 MHz. 사이클 수만 의미가 있다

    reg  [31:0]     words  [0:MAXW-1];
    reg  [15:0]     phases [0:MAXF-1];
    reg  [BITS-1:0] expv   [0:DEPTH-1];
    integer n_words, n_ph, n_exp;

    reg             rst_n = 1'b0;
    reg       [9:0] cx, cy;
    reg             clr = 1'b0;
    reg             rd_en = 1'b0;
    reg      [16:0] rd_addr = 17'd0;
    reg             feeding = 1'b0;
    integer         wi = 0, pi = 0, gap = 0, gap_left = 0;

    wire            in_ready, ph_ready, clr_busy, idle;
    wire [BITS-1:0] rd_data;
    wire     [15:0] frame_cnt;
    wire     [23:0] drop_cnt;
    wire            in_valid = feeding && (wi < n_words) && (gap_left == 0);
    wire     [31:0] in_word  = words[wi];
    wire            ph_valid = rst_n && (pi < n_ph);   // 리셋 동안에는 소스가 데이터를 내면 안 된다
    wire     [15:0] ph_data  = phases[pi];

`ifdef GLS
    wl_top dut (
`else
    wl_top #(.N(N), .M(M), .BITS(BITS), .CW(CW)) dut (
`endif
        .clk       (clk),
        .rst_n     (rst_n),
        .cx        (cx),
        .cy        (cy),
        .in_word   (in_word),
        .in_valid  (in_valid),
        .in_ready  (in_ready),
        .ph_data   (ph_data),
        .ph_valid  (ph_valid),
        .ph_ready  (ph_ready),
        .clr       (clr),
        .clr_busy  (clr_busy),
        .rd_en     (rd_en),
        .rd_x      (rd_addr % N),
        .rd_y      (rd_addr / N),
        .rd_data   (rd_data),
        .idle      (idle),
        .frame_cnt (frame_cnt),
        .drop_cnt  (drop_cnt)
    );

    // ------------------------------------------------------------ 자극 입력
    integer cycle = 0;
    always @(posedge clk) begin
        cycle <= cycle + 1;
        if (in_valid && in_ready) begin
            wi       <= wi + 1;
            gap_left <= gap;
        end else if (gap_left > 0) begin
            gap_left <= gap_left - 1;
        end
        if (ph_valid && ph_ready) pi <= pi + 1;
        // 감시 타이머: 없으면 파이프라인이 멈췄을 때 실행이 아무 말 없이 걸린다.
        if (cycle > 64 * n_words + 4 * DEPTH + 100000) begin
            $display("TIMEOUT at cycle %0d: word %0d / %0d, phase %0d / %0d, frames %0d, idle %0d",
                     cycle, wi, n_words, pi, n_ph, frame_cnt, idle);
`ifndef GLS
            $display("  cur_ok %0d nxt_ok %0d cordic busy %0d, decoder word %08x valid %0d, job valid %0d type %0d, s1 %0d x %0d",
                     dut.u_xf.cur_ok, dut.u_xf.nxt_ok, dut.u_xf.cd_busy, dut.u_dec.wd, dut.u_dec.wv,
                     dut.u_dec.j_valid, dut.u_dec.j_type, dut.u_xf.s1_v, dut.u_xf.xq_v);
`endif
            $display("RESULT FAIL");
            $finish;
        end
    end

    // 프레임당 사이클: 디코더를 떠나는 연속한 프레임 경계 사이의 시간.
    integer fc_last = 0, f_t0 = 0, f_max = 0, f_min = 0, f_sum = 0, f_n = 0, f_at = 0;
    always @(posedge clk) begin
        if (frame_cnt != fc_last) begin
            if ($test$plusargs("trace")) $display("frame %0d at cycle %0d, word %0d", frame_cnt, cycle, wi);
            fc_last <= frame_cnt;
            f_t0    <= cycle;
            if (fc_last > 0) begin       // 첫 프레임에는 리셋 / 시작 구간도 들어 있다
                if (cycle - f_t0 > f_max) begin
                    f_max <= cycle - f_t0;
                    f_at  <= frame_cnt;
                end
                if (f_n == 0 || cycle - f_t0 < f_min) f_min <= cycle - f_t0;
                f_sum <= f_sum + (cycle - f_t0);
                f_n   <= f_n + 1;
            end
        end
    end

    // 이벤트가 도는 동안의 SRAM 트래픽: 병합된 갱신 하나가 뱅크 읽기 한 번과 쓰기 한 번이다.
    // stall 은 wl_accum 이 이벤트를 받지 않은 클럭이다.
    integer n_ev = 0, n_upd = 0, n_stall = 0;
`ifndef GLS
    always @(posedge clk) if (feeding) begin
        if (dut.ev_valid && dut.ev_ready)  n_ev    <= n_ev + 1;
        if (dut.ev_valid && !dut.ev_ready) n_stall <= n_stall + 1;
        if (dut.u_acc.issue)               n_upd   <= n_upd + 1;
    end
`endif

    // 전력 분석용 활동 덤프: +vcd=<file> 은 첫 워드를 내준 뒤 +vcd_from=<n> 클럭부터
    // +vcd_len=<n> 클럭(기본 20000) 동안 설계의 모든 신호를 덤프한다. Verilator 는 VFLAGS 에
    // --trace 가 필요하다.
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

    // ------------------------------------------------------------ 읽어 내기 검사
    reg             rd_v = 1'b0;
    reg      [16:0] rd_a1 = 17'd0;
    integer         n_bad = 0, n_chk = 0, sum = 0, mx = 0, dump_fd = 0;
    always @(posedge clk) begin
        rd_v  <= rd_en;
        rd_a1 <= rd_addr;
        if (rd_v) begin
            n_chk <= n_chk + 1;
            sum   <= sum + rd_data;
            if (rd_data > mx) mx <= rd_data;
            if (dump_fd != 0) $fwrite(dump_fd, "%04x\n", rd_data);
            if (rd_data !== expv[rd_a1]) begin
                if (n_bad < 10)
                    $display("MISMATCH addr %0d (wx %0d wy %0d): got %0d expected %0d",
                             rd_a1, rd_a1 % N, rd_a1 / N, rd_data, expv[rd_a1]);
                n_bad <= n_bad + 1;
            end
        end
    end

    // ------------------------------------------------------------ 파일 읽기
    task load_hex32(input [8*512-1:0] path, output integer n);
        integer fd, r;
        reg [31:0] v;
        begin
            n  = 0;
            fd = $fopen(path, "r");
            if (fd == 0) begin
                $display("cannot open %0s", path);
                $finish;
            end
            r = $fscanf(fd, "%h\n", v);
            while (r == 1) begin
                if (n >= MAXW) begin
                    $display("more than MAXW = %0d words in %0s", MAXW, path);
                    $finish;
                end
                words[n] = v;
                n = n + 1;
                r = $fscanf(fd, "%h\n", v);
            end
            $fclose(fd);
        end
    endtask

    task load_hex16(input [8*512-1:0] path, input integer which, output integer n);
        integer fd, r;
        reg [31:0] v;
        begin
            n  = 0;
            fd = $fopen(path, "r");
            if (fd == 0) begin
                $display("cannot open %0s", path);
                $finish;
            end
            r = $fscanf(fd, "%h\n", v);
            while (r == 1) begin
                // 기댓값은 2^BITS - 1 로 자른다. 포화 카운터는 min(count, max) 에서 끝나고,
                // 골든 모델의 칸 폭이 CELL_BITS 일 때 골든 값 min(count, 2^CELL_BITS - 1) 을
                // 자르면 BITS <= CELL_BITS 인 어떤 BITS 에서도 같은 값이 된다. 더 넓은 칸
                // (예: 14비트 카운트)로 쓴 파일도 같은 방식으로 자른다.
                if (which == 0) phases[n] = v[15:0];
                else            expv[n]   = (v > {BITS{1'b1}}) ? {BITS{1'b1}} : v[BITS-1:0];
                n = n + 1;
                r = $fscanf(fd, "%h\n", v);
            end
            $fclose(fd);
        end
    endtask

    reg [8*512-1:0] vec, path, dump;
    integer a, t_start, t_end, tmp;
    initial begin
        if (!$value$plusargs("vec=%s", vec)) vec = "vec";
        if (!$value$plusargs("cx=%d", tmp)) tmp = 533;
        cx = tmp;
        if (!$value$plusargs("cy=%d", tmp)) tmp = 418;
        cy = tmp;
        if (!$value$plusargs("gap=%d", gap)) gap = 0;
        $sformat(path, "%0s/words.hex", vec);
        load_hex32(path, n_words);
        $sformat(path, "%0s/phase.hex", vec);
        load_hex16(path, 0, n_ph);
        $sformat(path, "%0s/expected.hex", vec);
        load_hex16(path, 1, n_exp);
        $display("loaded %0d words, %0d phases, %0d cells; centre (%0d, %0d), gap %0d",
                 n_words, n_ph, n_exp, cx, cy, gap);
        $fflush;
        if (n_exp != DEPTH) begin
            $display("expected.hex has %0d cells, design has %0d", n_exp, DEPTH);
            $finish;
        end
        if ($value$plusargs("dump=%s", dump)) dump_fd = $fopen(dump, "w");

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
        while (wi < n_words || !idle) @(posedge clk);
        t_end = cycle;
        feeding <= 1'b0;

        for (a = 0; a < DEPTH; a = a + 1) begin
            rd_en   <= 1'b1;
            rd_addr <= a;
            @(posedge clk);
        end
        rd_en <= 1'b0;
        repeat (3) @(posedge clk);
        if (dump_fd != 0) $fclose(dump_fd);

        $display("cycles %0d for %0d words; frames %0d (phases %0d); dropped %0d",
                 t_end - t_start, n_words, frame_cnt, n_ph, drop_cnt);
        if (f_n > 0)
            $display("cycles per frame: min %0d mean %0d max %0d (frame %0d), over %0d frames",
                     f_min, f_sum / f_n, f_max, f_at, f_n);
        $display("sram %0d updates for %0d events (%0.3f per event, merge width %0d), %0d accesses, %0d stall cycles",
                 n_upd, n_ev, n_upd * 1.0 / (n_ev > 0 ? n_ev : 1), CW, 2 * n_upd, n_stall);
        $display("memory sum %0d max %0d, checked %0d cells, mismatches %0d", sum, mx, n_chk, n_bad);
        if (n_bad == 0 && n_chk == DEPTH && frame_cnt == n_ph) $display("RESULT PASS");
        else $display("RESULT FAIL");
        $finish;
    end
endmodule
