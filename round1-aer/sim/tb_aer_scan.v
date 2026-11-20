`timescale 1ns/1ps
//======================================================================
// tb_aer_scan : B3 bench - synchronous scan readout (Samsung Gen3)
//
//   Mirrors tb_aer_rowcol so the three structures line up, but measures
//   ONE MORE THING, and that extra thing is the whole point of Gen3.
//
//   TWO DIFFERENT QUANTITIES
//
//     readout latency = out_req time - birth time
//         when the event physically leaves the chip.
//
//     TIMESTAMP ERROR = timestamp time - birth time
//         the error a downstream algorithm actually sees.
//
//   For an arbiter AER these are THE SAME NUMBER, because the timestamp
//   is applied as the event is read out. That identity is why the
//   arbiter curve in [S1] slide 13 climbs with event rate: readout
//   latency climbs, so timestamp error climbs with it.
//
//   Gen3 BREAKS the identity. Global hold stamps every pixel in the
//   array at one instant, and the scan that follows only decides WHEN
//   each event leaves - not what time it is labelled with. So:
//       readout latency  : still depends on scan position, up to a frame
//       timestamp error  : hold instant - birth, uniform in [0, T_frame)
//                          identical statistics for EVERY pixel,
//                          and INDEPENDENT OF EVENT RATE
//
//   Prediction to check: sigma_between of the timestamp error should be
//   ~0 for B3 (every pixel stamped at the same instant) even though the
//   readout ORDER is perfectly rigid - the most biased order possible.
//   That would say the artifact is removed not by making readout fair
//   but by decoupling the timestamp from readout, which is a different
//   and cheaper fix than a rotation-priority arbiter.
//
//   Cost, visible at the low-rate end: timestamp error is T_frame/2 on
//   average even when the array is nearly idle, where an arbiter answers
//   in tau_0. That is the low-activity penalty of [W] and R71.
//======================================================================
module tb_aer_scan;

  parameter integer ROW_D  = 3;
  parameter integer COL_D  = 3;
  parameter real    T_CLK  = 1.86;      // matches the async Delta
  parameter integer NBURST = 200;

  localparam integer NROW = (1 << ROW_D);
  localparam integer NCOL = (1 << COL_D);
  localparam integer NPX  = NROW * NCOL;
  localparam integer AW   = ROW_D + COL_D;

  localparam integer QD      = 16;
  localparam integer REQ_DLY = 4;
  localparam real    SINK_NS = 0.5;
  localparam integer ERR_PRINT_MAX = 20;

  //--------------------------------------------------------------------
  reg            rst_n;
  reg  [NPX-1:0] px_req;
  wire [NPX-1:0] px_ack;
  wire           out_req;
  wire [AW-1:0]  out_addr;
  reg            out_ack;
  wire           hold_pulse;

  aer_scan #(.ROW_D(ROW_D), .COL_D(COL_D), .T_CLK(T_CLK), .REQ_DLY(REQ_DLY)) dut (
    .rst_n(rst_n), .px_req(px_req), .px_ack(px_ack),
    .out_req(out_req), .out_addr(out_addr), .out_ack(out_ack),
    .hold_pulse(hold_pulse)
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

  // readout latency
  real lat_sum, lat_sq, lat_max, lat_min;
  real clat_sum[0:NPX-1], clat_sq[0:NPX-1];

  // TIMESTAMP error : measured against the global hold instant
  real ts_sum, ts_sq, ts_max;
  real cts_sum[0:NPX-1], cts_sq[0:NPX-1];

  // instant of the most recent global hold = the timestamp every event
  // read out in this frame will carry
  real t_hold;
  always @(posedge hold_pulse) t_hold = $realtime;

  integer total_done, err_cnt, k;
  reg     p0_pass, p1_pass, p2_pass, p3_pass, p1_stuck;
  real    res_cap;

  reg     burst_mode;
  integer burst_rank, burst_done;
  real    rank_sum[0:NPX-1], bts_sum[0:NPX-1];

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
        clat_sum[n]=0.0; clat_sq[n]=0.0; cts_sum[n]=0.0; cts_sq[n]=0.0;
      end
      lat_sum=0.0; lat_sq=0.0; lat_max=0.0; lat_min=1.0e30;
      ts_sum=0.0;  ts_sq=0.0;  ts_max=0.0;
      total_done=0;
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
  // ANOVA. mode 0 = readout latency, mode 1 = timestamp error.
  //--------------------------------------------------------------------
  task calc_stats;
    input  integer mode;
    output real m, sd, sdb, sdw;
    integer n, d;
    real vt, vb, vw, mi, tot, totsq, cs, cq;
    begin
      d = 0;
      for (n = 0; n < NPX; n = n + 1) d = d + done_cnt[n];
      tot   = (mode == 0) ? lat_sum : ts_sum;
      totsq = (mode == 0) ? lat_sq  : ts_sq;
      if (d == 0) begin m=0.0; sd=0.0; sdb=0.0; sdw=0.0; end
      else begin
        m  = tot/d;
        vt = totsq/d - m*m;
        vb = 0.0; vw = 0.0;
        for (n = 0; n < NPX; n = n + 1) if (done_cnt[n] > 0) begin
          cs = (mode == 0) ? clat_sum[n] : cts_sum[n];
          cq = (mode == 0) ? clat_sq[n]  : cts_sq[n];
          mi = cs/done_cnt[n];
          vb = vb + done_cnt[n]*(mi-m)*(mi-m);
          vw = vw + (cq - done_cnt[n]*mi*mi);
        end
        vb = vb/d; vw = vw/d;
        sd  = (vt > 0.0) ? $sqrt(vt) : 0.0;
        sdb = (vb > 0.0) ? $sqrt(vb) : 0.0;
        sdw = (vw > 0.0) ? $sqrt(vw) : 0.0;
      end
    end
  endtask

  //--------------------------------------------------------------------
  task sweep_point;
    input real r_meps;
    input real dur_ns;
    real    thr, lossp, rm, rsd, rsdb, rsdw, tm, tsd, tsdb, tsdw;
    integer n, g, a, l, d;
    begin
      gen_en = 1'b0;
      #3000;
      reset_counters;
      lambda_ch = (r_meps * 1.0e-3) / NPX;
      gen_en = 1'b1;
      #(dur_ns);
      gen_en = 1'b0;
      #3000;

      g=0; a=0; l=0; d=0;
      for (n = 0; n < NPX; n = n + 1) begin
        g=g+gen_cnt[n]; a=a+acc_cnt[n]; l=l+lost_cnt[n]; d=d+done_cnt[n];
      end
      thr   = (d * 1.0e3) / dur_ns;
      lossp = (g > 0) ? (100.0*l)/g : 0.0;
      calc_stats(0, rm, rsd, rsdb, rsdw);
      calc_stats(1, tm, tsd, tsdb, tsdw);
      if (thr > res_cap) res_cap = thr;

      $display("  %7.1f | %8.2f | %6.3f | %7.3f | %8.3f | %7.3f | %7.3f | %7.3f | %7.3f",
               r_meps, thr, lossp, rm, lat_max, tm, tsd, tsdb, tsdw);
    end
  endtask

  //--------------------------------------------------------------------
  integer p, before, i;

  initial begin : main
    rate_meps = 50.0;
    reset_counters; err_cnt=0; res_cap=0.0;
    p0_pass=0; p1_pass=0; p2_pass=0; p3_pass=0; p1_stuck=0; burst_mode=0;
    t_hold = 0.0;
    for (k = 0; k < NPX; k = k + 1) begin
      busy[k]=1'b0; rank_sum[k]=0.0; bts_sum[k]=0.0;
    end
    gen_en=1'b0; px_req={NPX{1'b0}}; out_ack=1'b0; rst_n=1'b0;

    $display("");
    $display("===== BUILD : B3 SCAN (Gen3) =====");
    $display(" array   : %0d x %0d = %0d px, %0d-bit address", NROW, NCOL, NPX, AW);
    $display(" T_CLK   : %0.3f ns/position -> frame = %0.1f ns, peak %0.1f Meps",
             T_CLK, NPX*T_CLK, 1000.0/T_CLK);
    $display(" arbiter cells : 0   (scanner, no mutex, no metastability)");
    $display("==================================");

    #20 rst_n = 1'b1;
    #10;

    // PHASE 0 ---------------------------------------------------------
    $display("\n---------- PHASE 0 : IDLE ----------");
    before = total_done;
    #(4*NPX*T_CLK);
    p0_pass = (total_done == before) && (err_cnt == 0);
    $display("PHASE 0 %s : %0d spurious output(s) over 4 frames",
             p0_pass ? "PASS" : "FAIL", total_done-before);

    // PHASE 1 ---------------------------------------------------------
    $display("\n---------- PHASE 1 : DIRECTED ----------");
    for (p = 0; p < NPX; p = p + 1) begin
      before = total_done;
      fire_event(p);
      fork : wait_p1
        begin wait (total_done == before + 1); disable wait_p1; end
        begin #(4*NPX*T_CLK + 2000);
              $display("PHASE 1 STUCK : px %0d no output", p);
              p1_stuck = 1'b1; disable wait_p1;
        end
      join
      if (p1_stuck) p = NPX;
      wait_idle;
    end
    p1_pass = (err_cnt == 0) && !p1_stuck;
    $display("PHASE 1 %s : %0d px, %0d error(s)",
             p1_pass ? "PASS" : "FAIL", NPX, err_cnt);

    // PHASE 2 ---------------------------------------------------------
    $display("\n---------- PHASE 2 : BURST (all %0d px) ----------", NPX);
    wait_idle; before = total_done;
    for (p = 0; p < NPX; p = p + 1) fire_event(p);
    fork : wait_b2
      begin wait (total_done == before + NPX); disable wait_b2; end
      begin #(6*NPX*T_CLK + 5000);
            $display("PHASE 2 FAIL : only %0d/%0d", total_done-before, NPX);
            disable wait_b2; end
    join
    p2_pass = (total_done == before + NPX);
    if (p2_pass) $display("PHASE 2 PASS : all %0d delivered in one frame", NPX);

    // PHASE 3/4 -------------------------------------------------------
    $display("\n---------- PHASE 4 : SWEEP (T4 axis) ----------");
    $display("  offered |  thruput |  loss  | rd_mean | rd_worst | ts_mean |  ts_STD | ts_bet  | ts_wit");
    $display("   [Meps] |   [Meps] |   [%%]  |   [ns]  |   [ns]   |   [ns]  |   [ns]  |   [ns]  |  [ns]");
    $display("  --------+----------+--------+---------+----------+---------+---------+---------+--------");
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
    p3_pass = (err_cnt == 0);
    $display("");
    $display("SCALING(B3 scan)  %0dx%0d=%0d px  T_CLK=%0.3f ns  frame=%0.1f ns  capacity=%0.2f Meps",
             NROW, NCOL, NPX, T_CLK, NPX*T_CLK, res_cap);

    // PHASE 5 ---------------------------------------------------------
    phase5;

    $display("\n============ B3 VERDICT ============");
    $display("   PHASE 0 IDLE     : %s", p0_pass ? "PASS" : "FAIL");
    $display("   PHASE 1 DIRECTED : %s", p1_pass ? "PASS" : "FAIL");
    $display("   PHASE 2 BURST    : %s", p2_pass ? "PASS" : "FAIL");
    $display("   PHASE 4 SWEEP    : %s", p3_pass ? "PASS" : "FAIL");
    $display("   --------------------------------");
    $display("   OVERALL : %s",
             (p0_pass && p1_pass && p2_pass && p3_pass) ? "PASS" : "FAIL");
    $display("====================================");
    $finish;
  end

  //--------------------------------------------------------------------
  // PHASE 5 : readout ORDER is rigid, but is the TIMESTAMP biased?
  //--------------------------------------------------------------------
  task phase5;
    integer n, i2, nok;
    real rmin, rmax, tmin, tmax, rv, tv;
    begin
      $display("\n---------- PHASE 5 : BURST FAIRNESS (P1) ----------");
      gen_en=1'b0; wait_idle; #3000; reset_counters;
      for (n = 0; n < NPX; n = n + 1) begin rank_sum[n]=0.0; bts_sum[n]=0.0; end

      nok = 0;
      for (i2 = 0; i2 < NBURST; i2 = i2 + 1) begin
        wait_idle;
        burst_rank=0; burst_done=0; burst_mode=1'b1;
        for (n = 0; n < NPX; n = n + 1) fire_event(n);
        fork : wb
          begin wait (burst_done == NPX); disable wb; end
          begin #(6*NPX*T_CLK + 5000); disable wb; end
        join
        burst_mode=1'b0;
        if (burst_done == NPX) nok = nok + 1;
        else begin
          $display(" burst %0d incomplete (%0d/%0d)", i2, burst_done, NPX);
          i2 = NBURST;
        end
      end
      if (nok == 0) begin $display(" no complete burst"); disable phase5; end
      $display(" completed bursts : %0d / %0d", nok, NBURST);

      rmin=1.0e30; rmax=-1.0; tmin=1.0e30; tmax=-1.0e30;
      for (n = 0; n < NPX; n = n + 1) begin
        rv = rank_sum[n]/nok;  tv = bts_sum[n]/nok;
        if (rv < rmin) rmin = rv;  if (rv > rmax) rmax = rv;
        if (tv < tmin) tmin = tv;  if (tv > tmax) tmax = tv;
      end
      $display(" readout RANK   min/max %0.2f / %0.2f  -> spread %0.2f (%0.1f %% of NPX)",
               rmin, rmax, rmax-rmin, 100.0*(rmax-rmin)/NPX);
      $display(" TIMESTAMP err  min/max %0.4f / %0.4f ns -> spread %0.4f ns",
               tmin, tmax, tmax-tmin);
      $display("   rank spread large + timestamp spread ~0 means the readout");
      $display("   order is maximally rigid yet carries NO spatial artifact,");
      $display("   because global hold decouples the timestamp from readout.");
    end
  endtask

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
    integer a; real lat, tse, birth;
    a = out_addr;
    if (bq_wr[a] == bq_rd[a]) begin
      if (err_cnt < ERR_PRINT_MAX)
        $display("[%0t] ERROR: output claims px %0d with no pending event", $realtime, a);
      else if (err_cnt == ERR_PRINT_MAX)
        $display("[%0t] ... further address errors suppressed", $realtime);
      err_cnt = err_cnt + 1;
    end else begin
      birth = bq[a*QD + (bq_rd[a] % QD)];
      lat   = $realtime - birth;      // readout latency
      tse   = t_hold   - birth;       // TIMESTAMP error : stamped at hold
      if (tse < 0.0) tse = 0.0;
      bq_rd[a]    = bq_rd[a] + 1;
      done_cnt[a] = done_cnt[a] + 1;
      total_done  = total_done + 1;

      lat_sum = lat_sum + lat;  lat_sq = lat_sq + lat*lat;
      clat_sum[a] = clat_sum[a] + lat;  clat_sq[a] = clat_sq[a] + lat*lat;
      if (lat > lat_max) lat_max = lat;
      if (lat < lat_min) lat_min = lat;

      ts_sum = ts_sum + tse;    ts_sq  = ts_sq  + tse*tse;
      cts_sum[a] = cts_sum[a] + tse;    cts_sq[a] = cts_sq[a] + tse*tse;
      if (tse > ts_max) ts_max = tse;

      if (burst_mode) begin
        rank_sum[a] = rank_sum[a] + burst_rank;
        bts_sum[a]  = bts_sum[a]  + tse;
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
