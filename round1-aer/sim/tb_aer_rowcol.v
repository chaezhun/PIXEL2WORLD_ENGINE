`timescale 1ns/1ps
//======================================================================
// tb_aer_rowcol : row-column bench, deliberately mirroring tb_aer_tree
//
//   Same phases, same stimulus model, same metrics as tb_aer_tree, so
//   the two structures can be put side by side. With
//       aer_tree    #(.DEPTH(6))              -> 64 channels, 6-bit addr
//       aer_rowcol  #(.ROW_D(3), .COL_D(3))   -> 8x8 = 64 px, 6-bit addr
//   the source count, the address width and the output stage are
//   identical, so any difference is caused by the ARBITRATION
//   STRUCTURE alone:
//
//       flat micropipeline tree : 63 mutex cells, pipelined
//       row/column arbiter      : 7 + 7 = 14 mutex cells, not pipelined
//                                 (4.5x fewer cells at 64 sources;
//                                  32x fewer at 64x64)
//
//   The expected trade is Boahen's: row/column buys area N -> sqrt(N)
//   and pays with a row arbitration AND a column arbitration per event,
//   over a single shared column resource that cannot pipeline.
//
//   PIXEL MODEL. Unlike the tree bench, a pixel is NOT a four-phase
//   source: it raises px_req and HOLDS it until acknowledged, which is
//   how a real DVS front-end behaves and what makes the shared column
//   line work.
//======================================================================
module tb_aer_rowcol;

  parameter integer ROW_D  = 3;
  parameter integer COL_D  = 3;
  parameter integer NBURST = 200;

  localparam integer NROW = (1 << ROW_D);
  localparam integer NCOL = (1 << COL_D);
  localparam integer NPX  = NROW * NCOL;
  localparam integer AW   = ROW_D + COL_D;

  localparam integer QD      = 16;
  localparam integer REQ_DLY = 4;
  localparam real    SINK_NS = 0.5;
  localparam integer ERR_PRINT_MAX = 20;

  parameter real C_TPD = 0.020;
  parameter real M_TPD = 0.020;

  // arbiter cell counts, for the area line in the report
  localparam integer CELLS_RC   = (NROW-1) + (NCOL-1);
  localparam integer CELLS_FLAT = NPX - 1;

  //--------------------------------------------------------------------
  reg            rst_n;
  reg  [NPX-1:0] px_req;
  wire [NPX-1:0] px_ack;
  wire           out_req;
  wire [AW-1:0]  out_addr;
  reg            out_ack;

  aer_rowcol #(
    .ROW_D(ROW_D), .COL_D(COL_D),
    .REQ_DLY(REQ_DLY), .C_TPD(C_TPD), .M_TPD(M_TPD)
  ) dut (
    .rst_n(rst_n), .px_req(px_req), .px_ack(px_ack),
    .out_req(out_req), .out_addr(out_addr), .out_ack(out_ack)
  );

  //--------------------------------------------------------------------
  real    rate_meps, lambda_ch;
  reg     gen_en;

  integer gen_cnt [0:NPX-1];
  integer acc_cnt [0:NPX-1];
  integer lost_cnt[0:NPX-1];
  integer done_cnt[0:NPX-1];
  reg     busy    [0:NPX-1];

  real    bq   [0:NPX*QD-1];
  integer bq_wr[0:NPX-1];
  integer bq_rd[0:NPX-1];

  real    lat_sum, lat_max, lat_min;

  //--------------------------------------------------------------------
  // TIMESTAMP ERROR (T4)
  //
  //   The timestamp is applied at READOUT, so its error is the readout
  //   latency. A CONSTANT latency is not an error - it is an offset and
  //   calibrates out. What survives is the SPREAD, which is what [S2]
  //   slide 30 plots as "Timestamp Error (STD)".
  //
  //   The spread decomposes, and the split is the useful part:
  //     sigma_total^2 = sigma_between^2 + sigma_within^2
  //       between : variance of the per-channel MEANS
  //                 = a fixed spatial pattern, removable by fair
  //                   arbitration (see the bit-reversal bias, PHASE 5)
  //       within  : variance inside each channel
  //                 = queueing noise, irreducible for a given Delta
  //   [S1] slide 13 shows only the total. Separating the two says how
  //   much of the artifact is a design choice and how much is physics.
  //--------------------------------------------------------------------
  real    lat_sq;                       // sum of squares, all channels
  real    clat_sum [0:NPX-1];
  real    clat_sq  [0:NPX-1];
  integer total_done, err_cnt, k;

  reg p0_pass, p1_pass, p2_pass, p3_pass, p1_stuck, sweep_dead;
  real res_tau0, res_cap, res_delta, res_worst;

  reg     burst_mode;
  integer burst_rank, burst_done;
  real    rank_sum[0:NPX-1];
  real    blat_sum[0:NPX-1];

  //--------------------------------------------------------------------
  // B1 vs B2 : ADDRESS ENCODING, not arbitration
  //
  //   A plain arbiter drains a row either way - the row line stays high
  //   while that row still has pending pixels, so the same row keeps
  //   winning. What separates word-parallel from word-serial burst-mode
  //   (Boahen, TCAS-I 51(7), 2004) is how many bits leave the chip:
  //
  //     word-parallel : ROW_D + COL_D  bits every event
  //     word-serial   : ROW_D once per burst, then COL_D per event
  //                     -> COL_D + ROW_D/k   bits/event
  //
  //   k = events per row grant, and k grows with event density, so the
  //   saving is scene dependent. This is [S2]'s 32 -> 12 -> 8 bit/event
  //   story, measured on the traffic used here.
  //--------------------------------------------------------------------
  integer blen_sum, blen_n, cur_row, cur_len;

  task blen_reset; begin blen_sum=0; blen_n=0; cur_row=-1; cur_len=0; end endtask
  task blen_close; begin
    if (cur_len > 0) begin blen_sum = blen_sum + cur_len; blen_n = blen_n + 1; end
    cur_row = -1; cur_len = 0;
  end endtask

  integer capR, fwdR;
  always @(posedge dut.cin)     capR = capR + 1;
  always @(posedge dut.out_req) fwdR = fwdR + 1;
  task clear_probes; begin capR=0; fwdR=0; end endtask

  //--------------------------------------------------------------------
  task fire_event(input integer p);
    begin
      gen_cnt[p] = gen_cnt[p] + 1;
      if (busy[p]) lost_cnt[p] = lost_cnt[p] + 1;
      else begin
        busy[p]                    = 1'b1;
        acc_cnt[p]                 = acc_cnt[p] + 1;
        bq[p*QD + (bq_wr[p] % QD)] = $realtime;
        bq_wr[p]                   = bq_wr[p] + 1;
        px_req[p]                  = 1'b1;
      end
    end
  endtask

  task reset_counters;
    integer n;
    begin
      for (n = 0; n < NPX; n = n + 1) begin
        gen_cnt[n]=0; acc_cnt[n]=0; lost_cnt[n]=0; done_cnt[n]=0;
        bq_wr[n]=0;   bq_rd[n]=0;
      end
      lat_sum=0.0; lat_max=0.0; lat_min=1.0e30; total_done=0;
      lat_sq=0.0;
      for (n = 0; n < NPX; n = n + 1) begin clat_sum[n]=0.0; clat_sq[n]=0.0; end
    end
  endtask

  task wait_idle;
    integer n, guard; reg any;
    begin
      guard=0; any=1'b1;
      while (any && guard < 20000) begin
        any = 1'b0;
        for (n = 0; n < NPX; n = n + 1) if (busy[n]) any = 1'b1;
        if (any) #10;
        guard = guard + 1;
      end
    end
  endtask

  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  // one-pass ANOVA on the latency samples
  //--------------------------------------------------------------------
  task calc_stats;
    output real m, sd, sdb, sdw;
    integer n, d;
    real vt, vb, vw, mi;
    begin
      d = 0;
      for (n = 0; n < NPX; n = n + 1) d = d + done_cnt[n];
      if (d == 0) begin m=0.0; sd=0.0; sdb=0.0; sdw=0.0; end
      else begin
        m  = lat_sum/d;
        vt = lat_sq/d - m*m;
        vb = 0.0; vw = 0.0;
        for (n = 0; n < NPX; n = n + 1) if (done_cnt[n] > 0) begin
          mi = clat_sum[n]/done_cnt[n];
          vb = vb + done_cnt[n]*(mi-m)*(mi-m);
          vw = vw + (clat_sq[n] - done_cnt[n]*mi*mi);
        end
        vb = vb/d; vw = vw/d;
        sd  = (vt > 0.0) ? $sqrt(vt) : 0.0;
        sdb = (vb > 0.0) ? $sqrt(vb) : 0.0;
        sdw = (vw > 0.0) ? $sqrt(vw) : 0.0;
      end
    end
  endtask

  task sweep_point;
    input real r_meps;
    input real dur_ns;
    real    thr, lossp, mean_lat, sd, sdb, sdw, kbar, bits_wp, bits_ws;
    integer n, g, a, l, d, inflight;
    begin
     if (!sweep_dead) begin
      gen_en = 1'b0;
      #2000;
      reset_counters;
      blen_reset;
      lambda_ch = (r_meps * 1.0e-3) / NPX;
      gen_en = 1'b1;
      #(dur_ns);
      gen_en = 1'b0;
      #3000;

      g=0; a=0; l=0; d=0;
      for (n = 0; n < NPX; n = n + 1) begin
        g=g+gen_cnt[n]; a=a+acc_cnt[n]; l=l+lost_cnt[n]; d=d+done_cnt[n];
      end
      inflight = a - d;
      thr      = (d * 1.0e3) / dur_ns;
      lossp    = (g > 0) ? (100.0*l)/g : 0.0;
      calc_stats(mean_lat, sd, sdb, sdw);

      blen_close;
      kbar   = (blen_n > 0) ? (1.0*blen_sum)/blen_n : 1.0;
      bits_wp = AW;
      bits_ws = COL_D + (1.0*ROW_D)/kbar;
      $display("  %7.1f | %8.2f | %6.3f | %7.3f | %7.3f | %7.3f | %7.3f | %8.3f | %5.2f | %5.2f | %s",
               r_meps, thr, lossp, mean_lat, sd, sdb, sdw, lat_max,
               kbar, bits_ws,
               (inflight <= NPX-1) ? "ok" : "LOSS");

      if (thr > res_cap)       res_cap   = thr;
      if (lat_max > res_worst) res_worst = lat_max;
      if (lat_min < res_tau0)  res_tau0  = lat_min;

      if (d == 0) begin
        sweep_dead = 1'b1;
        $display("  -> zero completions: DUT wedged, skipping remaining points");
      end
     end
    end
  endtask

  //--------------------------------------------------------------------
  integer p, before, i;

  initial begin : main
    rate_meps = 50.0;
    reset_counters; blen_reset;
    err_cnt=0;
    p0_pass=0; p1_pass=0; p2_pass=0; p3_pass=0; p1_stuck=0; sweep_dead=0;
    burst_mode=0;
    res_tau0=1.0e30; res_cap=0.0; res_worst=0.0;
    for (k = 0; k < NPX; k = k + 1) begin
      busy[k]=1'b0; rank_sum[k]=0.0; blat_sum[k]=0.0;
    end
    gen_en=1'b0; px_req={NPX{1'b0}}; out_ack=1'b0; rst_n=1'b0;

    $display("");
    $display("===== BUILD : B1 ROW/COLUMN =====");
`ifdef AER_STRUCT_MUTEX
    $display(" mutex       : STRUCTURAL");
`else
    $display(" mutex       : behavioural");
`endif
    $display(" array       : %0d x %0d = %0d px, %0d-bit address",
             NROW, NCOL, NPX, AW);
    $display(" arbiter cells : row/col %0d   vs flat tree %0d   (%0.1fx fewer)",
             CELLS_RC, CELLS_FLAT, (1.0*CELLS_FLAT)/CELLS_RC);
    $display(" C_TPD %0.3f / M_TPD %0.3f / REQ_DLY %0d / SINK %0.2f ns",
             C_TPD, M_TPD, REQ_DLY, SINK_NS);
    $display("=================================");

    #20 rst_n = 1'b1;
    #10;

    // PHASE 0 ---------------------------------------------------------
    $display("\n---------- PHASE 0 : IDLE ----------");
    clear_probes; #500;
    p0_pass = (capR == 0) && (fwdR == 0);
    $display("PHASE 0 %s : root cap/fwd = %0d/%0d",
             p0_pass ? "PASS" : "FAIL (self-oscillating)", capR, fwdR);

    // PHASE 1 ---------------------------------------------------------
    $display("\n---------- PHASE 1 : DIRECTED ----------");
    for (p = 0; p < NPX; p = p + 1) begin
      before = total_done;
      fire_event(p);
      fork : wait_p1
        begin wait (total_done == before + 1); disable wait_p1; end
        begin #2000;
              $display("PHASE 1 STUCK : pixel %0d (row %0d col %0d) no output in 2 us",
                       p, p/NCOL, p%NCOL);
              p1_stuck = 1'b1; disable wait_p1;
        end
      join
      if (p1_stuck) p = NPX;
      #20;
    end
    p1_pass = (err_cnt == 0) && !p1_stuck;
    $display("PHASE 1 %s : %0d pixel(s), %0d error(s)",
             p1_pass ? "PASS" : "FAIL", NPX, err_cnt);
    #100;

    // PHASE 2 ---------------------------------------------------------
    $display("\n---------- PHASE 2 : BURST (all %0d px) ----------", NPX);
    before = total_done; clear_probes;
    for (p = 0; p < NPX; p = p + 1) fire_event(p);
    fork : wait_burst
      begin wait (total_done == before + NPX); disable wait_burst; end
      begin #(400*NPX + 20000);
            $display("PHASE 2 FAIL : only %0d/%0d delivered", total_done-before, NPX);
            disable wait_burst;
      end
    join
    p2_pass = (total_done == before + NPX);
    if (p2_pass) $display("PHASE 2 PASS : all %0d contending events delivered", NPX);
    $display(" out cap/fwd = %0d/%0d   [expect %0d/%0d]", capR, fwdR, NPX, NPX);
    #500;

    // PHASE 3 ---------------------------------------------------------
    $display("\n---------- PHASE 3 : POISSON @ %0.0f Meps ----------", rate_meps);
    wait_idle; #1000;
    reset_counters; err_cnt=0; clear_probes;
    lambda_ch = (rate_meps * 1.0e-3) / NPX;
    gen_en=1'b1; #200000; gen_en=1'b0; #3000;
    report3;

    // PHASE 4 ---------------------------------------------------------
    $display("\n---------- PHASE 4 : CAPACITY SWEEP ----------");
    $display("  word-parallel = %0d bit/event (fixed).  word-serial = COL_D + ROW_D/k", AW);
    $display("  offered |  thruput |  loss  |   mean  |  STD    | STD_bet | STD_wit |   worst  |  kbar | b/ev");
    $display("   [Meps] |   [Meps] |   [%%]  |   [ns]  |   [ns]  |   [ns]  |   [ns]  |   [ns]   |  (B2) | (B2)");
    $display("  --------+----------+--------+---------+---------+---------+---------+----------+-------+-----");
    // [S1] slide 13 axis: 1 ~ 1000 Meps, Poisson. Window scaled so
    // every point collects ~3000 events (STD accurate to ~1.3 %).
    sweep_point(   1.0, 3000000.0);
    sweep_point(   2.0, 1500000.0);
    sweep_point(   5.0,  600000.0);
    sweep_point(  10.0,  300000.0);
    sweep_point(  20.0,  150000.0);
    sweep_point(  50.0,  100000.0);
    sweep_point( 100.0,   60000.0);
    sweep_point( 200.0,   60000.0);
    sweep_point( 300.0,   50000.0);
    sweep_point( 400.0,   50000.0);
    sweep_point( 500.0,   50000.0);
    sweep_point( 600.0,   50000.0);
    sweep_point( 700.0,   50000.0);
    sweep_point( 850.0,   50000.0);
    sweep_point(1000.0,   50000.0);

    res_delta = (res_cap > 0.0) ? 1000.0/res_cap : 0.0;
    $display("");
    $display("SCALING(B1 row/col)  %0dx%0d=%0d px  tau0=%0.3f ns  capacity=%0.2f Meps  Delta=%0.3f ns  worst=%0.3f ns",
             NROW, NCOL, NPX, res_tau0, res_cap, res_delta, res_worst);
    $display("  arbiter cells %0d (flat tree would need %0d)", CELLS_RC, CELLS_FLAT);

    // PHASE 5 ---------------------------------------------------------
    phase5_fairness;

    $display("\n============ B1 VERDICT ============");
    $display("   PHASE 0 IDLE     : %s", p0_pass ? "PASS" : "FAIL");
    $display("   PHASE 1 DIRECTED : %s", p1_pass ? "PASS" : "FAIL");
    $display("   PHASE 2 BURST    : %s", p2_pass ? "PASS" : "FAIL");
    $display("   PHASE 3 POISSON  : %s", p3_pass ? "PASS" : "FAIL");
    $display("   --------------------------------");
    $display("   OVERALL : %s",
             (p0_pass && p1_pass && p2_pass && p3_pass) ? "PASS" : "FAIL");
    $display("====================================");
    $finish;
  end

  //--------------------------------------------------------------------
  task phase5_fairness;
    integer n, i2, nok;
    real ideal, rmin, rmax, rv;
    begin
      $display("\n---------- PHASE 5 : BURST FAIRNESS (P1) ----------");
      $display(" %0d synchronised bursts of %0d px. fair -> every px %0.1f",
               NBURST, NPX, (NPX-1)/2.0);
      gen_en=1'b0; #2000; reset_counters;
      for (n = 0; n < NPX; n = n + 1) begin rank_sum[n]=0.0; blat_sum[n]=0.0; end

      nok = 0;
      for (i2 = 0; i2 < NBURST; i2 = i2 + 1) begin
        wait_idle;
        burst_rank=0; burst_done=0; burst_mode=1'b1;
        for (n = 0; n < NPX; n = n + 1) fire_event(n);
        fork : wait_b
          begin wait (burst_done == NPX); disable wait_b; end
          begin #(200*NPX + 10000); disable wait_b; end
        join
        burst_mode=1'b0;
        if (burst_done == NPX) nok = nok + 1;
        else begin
          $display(" burst %0d incomplete (%0d/%0d) - stopping", i2, burst_done, NPX);
          i2 = NBURST;
        end
        #200;
      end
      if (nok == 0) begin
        $display(" no complete burst - no fairness data");
        disable phase5_fairness;
      end
      $display(" completed bursts : %0d / %0d", nok, NBURST);

      ideal = (NPX-1)/2.0; rmin=1.0e30; rmax=-1.0;
      for (n = 0; n < NPX; n = n + 1) begin
        rv = rank_sum[n]/nok;
        if (rv < rmin) rmin = rv;
        if (rv > rmax) rmax = rv;
      end
      $display(" mean rank per pixel (row-major):");
      for (n = 0; n < NPX; n = n + 1) begin
        if (n % NCOL == 0) $write("  r%2d:", n/NCOL);
        $write(" %6.2f", rank_sum[n]/nok);
        if (n % NCOL == NCOL-1) $display("");
      end
      $display(" mean burst latency per pixel [ns]:");
      for (n = 0; n < NPX; n = n + 1) begin
        if (n % NCOL == 0) $write("  r%2d:", n/NCOL);
        $write(" %6.1f", blat_sum[n]/nok);
        if (n % NCOL == NCOL-1) $display("");
      end
      $display(" ideal %0.2f   observed min/max %0.2f / %0.2f", ideal, rmin, rmax);
      $display(" UNFAIRNESS SPREAD : %0.2f ranks (%0.1f %% of NPX)",
               rmax-rmin, 100.0*(rmax-rmin)/NPX);
    end
  endtask

  task report3;
    integer n, g, a, l, d; real thr, lossp;
    begin
      g=0;a=0;l=0;d=0;
      for (n = 0; n < NPX; n = n + 1) begin
        g=g+gen_cnt[n]; a=a+acc_cnt[n]; l=l+lost_cnt[n]; d=d+done_cnt[n];
      end
      thr   = (d*1.0e3)/200000.0;
      lossp = (g>0) ? (100.0*l)/g : 0.0;
      p3_pass = (err_cnt == 0) && ((a-d) <= NPX-1);
      $display(" generated/accepted/lost/done : %0d / %0d / %0d / %0d", g,a,l,d);
      $display(" loss %0.3f %%   throughput %0.3f Meps", lossp, thr);
      if (d > 0)
        $display(" latency mean %0.3f ns (%0.3f ns/event-bit)  min %0.3f  WORST %0.3f",
                 lat_sum/d, (lat_sum/d)/AW, lat_min, lat_max);
      $display(" out cap/fwd %0d/%0d   in-flight %0d / %0d   ERRORS %0d",
               capR, fwdR, a-d, NPX-1, err_cnt);
      if (a-d > NPX-1)
        $display(" *** SILENT LOSS: ~%0d acknowledged but never emitted", (a-d)-(NPX-1));
    end
  endtask

  //--------------------------------------------------------------------
  // pixels : hold px_req until acknowledged (NOT a four-phase source)
  //--------------------------------------------------------------------
  genvar gp;
  generate
    for (gp = 0; gp < NPX; gp = gp + 1) begin : g_px
      integer seed_i; real iat, u;
      initial begin
        seed_i = 32'h5EED0000 + gp*7919 + 1;
        forever begin
          wait (gen_en === 1'b1);
          u   = $dist_uniform(seed_i, 1, 1000000) / 1000000.0;
          iat = -$ln(u) / lambda_ch;
          #(iat);
          if (gen_en) fire_event(gp);
        end
      end
      initial begin
        forever begin
          @(posedge px_ack[gp]);
          px_req[gp] = 1'b0;
          @(negedge px_ack[gp]);
          busy[gp]   = 1'b0;
        end
      end
    end
  endgenerate

  //--------------------------------------------------------------------
  always @(posedge out_req) begin : b_meas
    integer a; real lat;
    a = out_addr;
    if (bq_wr[a] == bq_rd[a]) begin
      if (err_cnt < ERR_PRINT_MAX)
        $display("[%0t] ERROR: output claims px %0d (r%0d c%0d) with no pending event",
                 $realtime, a, a/NCOL, a%NCOL);
      else if (err_cnt == ERR_PRINT_MAX)
        $display("[%0t] ... further address errors suppressed", $realtime);
      err_cnt = err_cnt + 1;
    end else begin
      lat = $realtime - bq[a*QD + (bq_rd[a] % QD)];
      bq_rd[a]    = bq_rd[a] + 1;
      done_cnt[a] = done_cnt[a] + 1;
      total_done  = total_done + 1;
      lat_sum     = lat_sum + lat;
      lat_sq      = lat_sq  + lat*lat;
      clat_sum[a] = clat_sum[a] + lat;
      clat_sq [a] = clat_sq [a] + lat*lat;
      if (lat > lat_max) lat_max = lat;
      if (lat < lat_min) lat_min = lat;
      // burst length = run of consecutive outputs from the same row
      if ((a >> COL_D) == cur_row) cur_len = cur_len + 1;
      else begin
        if (cur_len > 0) begin blen_sum = blen_sum + cur_len; blen_n = blen_n + 1; end
        cur_row = (a >> COL_D); cur_len = 1;
      end
      if (burst_mode) begin
        rank_sum[a] = rank_sum[a] + burst_rank;
        blat_sum[a] = blat_sum[a] + lat;
        burst_rank  = burst_rank + 1;
        burst_done  = burst_done + 1;
      end
    end
  end

  initial begin
    forever begin
      @(posedge out_req);
      #(SINK_NS) out_ack = 1'b1;
      @(negedge out_req);
      #(SINK_NS) out_ack = 1'b0;
    end
  end

endmodule
