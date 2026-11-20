`timescale 1ns/1ps
//======================================================================
// tb_aer_tree : generic-depth testbench for aer_tree
//
//   Two jobs, both of which the fixed 8-input bench could not do.
//
//   (A) T2 SCALING.  Sweep DEPTH and measure, per depth:
//         tau_0     fixed path delay          (latency min at low load)
//         Delta     service quantum           (1 / saturated throughput)
//         capacity, worst-case latency bound
//       These are the entire input to R22's tile-splitting guidance
//         (N, lambda, Delta, tau_0) -> (N/M, lambda, Delta_t, tau_0,t)
//       Assuming the depth dependence instead of measuring it would make
//       every tile-count conclusion unfounded.
//
//   (B) P1 UNFAIR ARBITRATION.: under Poisson
//       arrivals a deterministic tie-break produced NO detectable bias,
//       because in continuous time exact ties have probability zero, so
//       the tie-break rule almost never fires. Arbitration order is then
//       set by arrival order, not by policy.
//
//       Unfairness therefore has to be measured with SYNCHRONISED
//       BURSTS - which is also the physically relevant case, since a
//       global illumination change makes many DVS pixels fire together.
//
//       The metric is SERVICE RANK, not event count: in a burst every
//       channel is eventually served, so counts are all 1 and carry no
//       information. Rank (0 = served first) is what turns into waiting
//       time, and waiting time is exactly the "artifacts and delay by
//       unfair arbitration" of Ryu, CVPR19 workshop, slide 10.
//         perfectly fair  -> every channel's mean rank = (NCH-1)/2
//         biased          -> low-index channels sit at consistently low
//                            rank and high-index channels starve upward
//
//   Run for one depth:
//     set_property generic {DEPTH=5} [get_filesets sim_1]
//   or sweep all of them from the Tcl console (see docs/03_RTL_README.md).
//======================================================================
module tb_aer_tree;

  parameter integer DEPTH   = 3;              // NCH = 2**DEPTH
  parameter integer NBURST  = 200;            // PHASE 5 repetitions

  localparam integer NCH     = (1 << DEPTH);
  localparam integer QD      = 16;            // per-channel birth queue
  localparam integer REQ_DLY = 4;
  localparam real    SINK_NS = 0.5;
  localparam integer ERR_PRINT_MAX = 20;

  parameter real C_TPD = 0.020;               // C-element delay [ns]
  parameter real M_TPD = 0.020;               // mutex base gate delay [ns]

`ifdef AER_STRUCT_MUTEX
  localparam real gnt_window = 2.0 * M_TPD;
`else
  localparam real gnt_window = 2.5 * M_TPD;
`endif

  //--------------------------------------------------------------------
  // DUT
  //--------------------------------------------------------------------
  reg              rst_n;
  reg  [NCH-1:0]   ev_req;
  wire [NCH-1:0]   ev_ack;
  wire             out_req;
  wire [DEPTH-1:0] out_addr;
  reg              out_ack;

  aer_tree #(
    .DEPTH   (DEPTH),
    .REQ_DLY (REQ_DLY),
    .C_TPD   (C_TPD),
    .M_TPD   (M_TPD)
  ) dut (
    .rst_n  (rst_n),
    .ev_req (ev_req),
    .ev_ack (ev_ack),
    .r_req  (out_req),
    .r_data (out_addr),
    .r_ack  (out_ack)
  );

  //--------------------------------------------------------------------
  // state
  //--------------------------------------------------------------------
  real    rate_meps, lambda_ch, sim_ns;
  reg     gen_en;

  integer gen_cnt  [0:NCH-1];
  integer acc_cnt  [0:NCH-1];
  integer lost_cnt [0:NCH-1];
  integer done_cnt [0:NCH-1];
  reg     busy     [0:NCH-1];

  real    bq    [0:NCH*QD-1];
  integer bq_wr [0:NCH-1];
  integer bq_rd [0:NCH-1];

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
  real    clat_sum [0:NCH-1];
  real    clat_sq  [0:NCH-1];
  integer total_done, err_cnt;
  integer k;

  reg p0_pass, p1_pass, p2_pass, p3_pass;
  reg p1_stuck, sweep_dead;

  // scaling results, printed as one line at the end
  real  res_tau0, res_cap, res_delta, res_worst;

  //--------------------------------------------------------------------
  // PHASE 5 state : synchronised-burst service rank
  //--------------------------------------------------------------------
  reg     burst_mode;
  integer burst_rank;                 // 0 = served first in this burst
  real    rank_sum  [0:NCH-1];
  integer rank_max  [0:NCH-1];
  real    blat_sum  [0:NCH-1];
  integer burst_done;

  //--------------------------------------------------------------------
  // root probe : conservation check without per-node plumbing
  //--------------------------------------------------------------------
  integer capR, fwdR;
  generate
    if (DEPTH > 1) begin : g_probe
      always @(posedge dut.g_node.u_n.cin)   capR = capR + 1;
      always @(posedge dut.g_node.u_n.r_req) fwdR = fwdR + 1;
    end else begin : g_probe1
      always @(posedge dut.g_leaf.u_n.cin)   capR = capR + 1;
      always @(posedge dut.g_leaf.u_n.r_req) fwdR = fwdR + 1;
    end
  endgenerate

  task clear_probes; begin capR = 0; fwdR = 0; end endtask

  // Block until every source has finished its four-phase. Firing a new
  // burst while a channel is still busy would silently drop that
  // channel (one-slot gating), so the burst could never complete and
  // every repetition would cost a full timeout.
  task wait_idle;
    integer n, guard;
    reg any;
    begin
      guard = 0;
      any   = 1'b1;
      while (any && guard < 10000) begin
        any = 1'b0;
        for (n = 0; n < NCH; n = n + 1) if (busy[n]) any = 1'b1;
        if (any) #10;
        guard = guard + 1;
      end
    end
  endtask

  //--------------------------------------------------------------------
  // helpers
  //--------------------------------------------------------------------
  task fire_event(input integer ch);
    begin
      gen_cnt[ch] = gen_cnt[ch] + 1;
      if (busy[ch]) lost_cnt[ch] = lost_cnt[ch] + 1;
      else begin
        busy[ch]                     = 1'b1;
        acc_cnt[ch]                  = acc_cnt[ch] + 1;
        bq[ch*QD + (bq_wr[ch] % QD)] = $realtime;
        bq_wr[ch]                    = bq_wr[ch] + 1;
        ev_req[ch]                   = 1'b1;
      end
    end
  endtask

  task reset_counters;
    integer n;
    begin
      for (n = 0; n < NCH; n = n + 1) begin
        gen_cnt[n]=0; acc_cnt[n]=0; lost_cnt[n]=0; done_cnt[n]=0;
        bq_wr[n]=0;   bq_rd[n]=0;
      end
      lat_sum=0.0; lat_max=0.0; lat_min=1.0e30; total_done=0;
      lat_sq=0.0;
      for (n = 0; n < NCH; n = n + 1) begin clat_sum[n]=0.0; clat_sq[n]=0.0; end
    end
  endtask

  //--------------------------------------------------------------------
  // one point on the capacity curve
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
      for (n = 0; n < NCH; n = n + 1) d = d + done_cnt[n];
      if (d == 0) begin m=0.0; sd=0.0; sdb=0.0; sdw=0.0; end
      else begin
        m  = lat_sum/d;
        vt = lat_sq/d - m*m;
        vb = 0.0; vw = 0.0;
        for (n = 0; n < NCH; n = n + 1) if (done_cnt[n] > 0) begin
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
    real    thr, lossp, mean_lat, sd, sdb, sdw;
    integer n, g, a, l, d, inflight;
    begin
     if (!sweep_dead) begin
      gen_en = 1'b0;
      #2000;
      reset_counters;
      lambda_ch = (r_meps * 1.0e-3) / NCH;

      gen_en = 1'b1;
      #(dur_ns);
      gen_en = 1'b0;
      #2000;                                   // drain

      g=0; a=0; l=0; d=0;
      for (n = 0; n < NCH; n = n + 1) begin
        g = g + gen_cnt[n];  a = a + acc_cnt[n];
        l = l + lost_cnt[n]; d = d + done_cnt[n];
      end
      inflight = a - d;
      thr      = (d * 1.0e3) / dur_ns;
      lossp    = (g > 0) ? (100.0*l)/g : 0.0;
      calc_stats(mean_lat, sd, sdb, sdw);

      $display("  %7.1f | %8.2f | %6.3f | %7.3f | %7.3f | %7.3f | %7.3f | %8.3f | %s",
               r_meps, thr, lossp, mean_lat, sd, sdb, sdw, lat_max,
               (inflight <= NCH-1) ? "ok" : "LOSS");

      // carry the saturated point out for the scaling summary
      if (thr > res_cap)   res_cap   = thr;
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
  // main
  //--------------------------------------------------------------------
  integer ch, before, b;

  initial begin : main
    rate_meps = 50.0;
    reset_counters;
    err_cnt = 0;
    p0_pass=1'b0; p1_pass=1'b0; p2_pass=1'b0; p3_pass=1'b0;
    p1_stuck=1'b0; sweep_dead=1'b0; burst_mode=1'b0;
    res_tau0 = 1.0e30; res_cap = 0.0; res_worst = 0.0;
    for (k = 0; k < NCH; k = k + 1) begin
      busy[k]=1'b0; rank_sum[k]=0.0; rank_max[k]=0; blat_sum[k]=0.0;
    end

    gen_en  = 1'b0;
    ev_req  = {NCH{1'b0}};
    out_ack = 1'b0;
    rst_n   = 1'b0;

    $display("");
    $display("===== BUILD =====");
`ifdef AER_STRUCT_C
    $display(" c_element   : STRUCTURAL");
`else
    $display(" c_element   : behavioural");
`endif
`ifdef AER_STRUCT_MUTEX
    $display(" mutex       : STRUCTURAL, window = %0.3f ns", gnt_window);
`else
    $display(" mutex       : behavioural, window = %0.3f ns", gnt_window);
`endif
`ifdef AER_STRUCT_DLY
    $display(" delay_chain : STRUCTURAL");
`else
    $display(" delay_chain : behavioural");
`endif
    $display(" DEPTH %0d -> NCH %0d   C_TPD %0.3f / M_TPD %0.3f / REQ_DLY %0d / SINK %0.2f ns",
             DEPTH, NCH, C_TPD, M_TPD, REQ_DLY, SINK_NS);
    if (gnt_window > C_TPD)
      $display(" margin check: window %0.3f ns > C-element %0.3f ns -> ok", gnt_window, C_TPD);
    else
      $display(" margin check: window %0.3f ns <= C-element %0.3f ns -> TOO TIGHT", gnt_window, C_TPD);
    $display("=================");

    #20 rst_n = 1'b1;
    #10;

    //================================================================
    // PHASE 0 : idle
    //================================================================
    $display("");
    $display("---------- PHASE 0 : IDLE ----------");
    clear_probes;
    #500;
    p0_pass = (capR == 0) && (fwdR == 0);
    if (p0_pass) $display("PHASE 0 PASS : idle, no spurious activity");
    else         $display("PHASE 0 FAIL : self-oscillating, root cap/fwd = %0d/%0d", capR, fwdR);

    //================================================================
    // PHASE 1 : directed
    //================================================================
    $display("\n---------- PHASE 1 : DIRECTED ----------");
    for (ch = 0; ch < NCH; ch = ch + 1) begin
      before = total_done;
      fire_event(ch);
      fork : wait_p1
        begin wait (total_done == before + 1); disable wait_p1; end
        begin #2000;
              $display("PHASE 1 STUCK : channel %0d produced no output in 2 us", ch);
              p1_stuck = 1'b1; disable wait_p1;
        end
      join
      if (p1_stuck) ch = NCH;
      #20;
    end
    p1_pass = (err_cnt == 0) && !p1_stuck;
    $display("PHASE 1 %s : %0d channel(s), %0d error(s)",
             p1_pass ? "PASS" : "FAIL", NCH, err_cnt);
    #100;

    //================================================================
    // PHASE 2 : one synchronised burst
    //================================================================
    $display("\n---------- PHASE 2 : BURST ----------");
    before = total_done;
    clear_probes;
    for (ch = 0; ch < NCH; ch = ch + 1) fire_event(ch);
    fork : wait_burst
      begin wait (total_done == before + NCH); disable wait_burst; end
      begin #(200*NCH + 20000);
            $display("PHASE 2 FAIL : only %0d/%0d delivered", total_done-before, NCH);
            disable wait_burst;
      end
    join
    p2_pass = (total_done == before + NCH);
    if (p2_pass) $display("PHASE 2 PASS : all %0d contending events delivered", NCH);
    $display(" root cap/fwd = %0d/%0d   [expect %0d/%0d]", capR, fwdR, NCH, NCH);
    #500;

    //================================================================
    // PHASE 3 : Poisson, single point
    //================================================================
    $display("\n---------- PHASE 3 : POISSON @ %0.0f Meps ----------", rate_meps);
    #1000;
    reset_counters;
    err_cnt = 0;
    clear_probes;
    lambda_ch = (rate_meps * 1.0e-3) / NCH;
    gen_en = 1'b1;
    #200000;
    gen_en = 1'b0;
    #2000;
    report3;

    //================================================================
    // PHASE 4 : capacity sweep
    //================================================================
    $display("\n---------- PHASE 4 : CAPACITY SWEEP ----------");
    $display("  offered |  thruput |  loss  |   mean  |  STD    | STD_bet | STD_wit |   worst  |");
    $display("   [Meps] |   [Meps] |   [%%]  |   [ns]  |   [ns]  |   [ns]  |   [ns]  |   [ns]   |");
    $display("  --------+----------+--------+---------+---------+---------+---------+----------+");
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
    $display("SCALING  DEPTH=%0d  NCH=%0d  tau0=%0.3f ns  capacity=%0.2f Meps  Delta=%0.3f ns  worst=%0.3f ns",
             DEPTH, NCH, res_tau0, res_cap, res_delta, res_worst);

    //================================================================
    // PHASE 5 : P1 - unfair arbitration under synchronised bursts
    //================================================================
    phase5_fairness;

    //================================================================
    $display("\n================= T1 VERDICT =================");
    $display("   PHASE 0 IDLE     : %s", p0_pass ? "PASS" : "FAIL");
    $display("   PHASE 1 DIRECTED : %s", p1_pass ? "PASS" : "FAIL");
    $display("   PHASE 2 BURST    : %s", p2_pass ? "PASS" : "FAIL");
    $display("   PHASE 3 POISSON  : %s", p3_pass ? "PASS" : "FAIL");
    $display("   ------------------------------------------");
    $display("   OVERALL : %s",
             (p0_pass && p1_pass && p2_pass && p3_pass) ? "PASS" : "FAIL");
    $display("==============================================");
    $finish;
  end

  //--------------------------------------------------------------------
  // PHASE 5 : repeated synchronised bursts, per-channel SERVICE RANK
  //--------------------------------------------------------------------
  task phase5_fairness;
    integer n, i, nburst_ok;
    real    ideal, rmin, rmax, r;
    begin
      $display("\n---------- PHASE 5 : BURST FAIRNESS (P1) ----------");
      $display(" %0d synchronised bursts of %0d channels.", NBURST, NCH);
      $display(" rank 0 = served first. Perfectly fair -> every channel %0.1f",
               (NCH-1)/2.0);

      gen_en = 1'b0;
      #2000;
      reset_counters;
      for (n = 0; n < NCH; n = n + 1) begin
        rank_sum[n]=0.0; rank_max[n]=0; blat_sum[n]=0.0;
      end

      nburst_ok = 0;
      for (i = 0; i < NBURST; i = i + 1) begin
        wait_idle;
        burst_rank = 0;
        burst_done = 0;
        burst_mode = 1'b1;
        for (n = 0; n < NCH; n = n + 1) fire_event(n);
        fork : wait_b
          begin wait (burst_done == NCH); disable wait_b; end
          begin #(100*NCH + 5000); disable wait_b; end
        join
        burst_mode = 1'b0;
        if (burst_done == NCH) nburst_ok = nburst_ok + 1;
        else begin
          $display(" burst %0d incomplete (%0d/%0d) - stopping PHASE 5",
                   i, burst_done, NCH);
          i = NBURST;
        end
        #200;
      end

      if (nburst_ok == 0) begin
        $display(" PHASE 5 produced no complete burst - no fairness data");
        disable phase5_fairness;
      end
      $display(" completed bursts    : %0d / %0d", nburst_ok, NBURST);

      ideal = (NCH-1)/2.0;
      rmin  = 1.0e30; rmax = -1.0;
      for (n = 0; n < NCH; n = n + 1) begin
        r = rank_sum[n]/nburst_ok;
        if (r < rmin) rmin = r;
        if (r > rmax) rmax = r;
      end

      $display(" mean rank per channel:");
      for (n = 0; n < NCH; n = n + 1) begin
        if (n % 8 == 0) $write("  ch%3d:", n);
        $write(" %6.2f", rank_sum[n]/nburst_ok);
        if (n % 8 == 7) $display("");
      end
      if (NCH % 8 != 0) $display("");

      $display(" mean burst latency per channel [ns]:");
      for (n = 0; n < NCH; n = n + 1) begin
        if (n % 8 == 0) $write("  ch%3d:", n);
        $write(" %6.1f", blat_sum[n]/nburst_ok);
        if (n % 8 == 7) $display("");
      end
      if (NCH % 8 != 0) $display("");

      $display(" ideal mean rank      : %0.2f", ideal);
      $display(" observed min / max   : %0.2f / %0.2f", rmin, rmax);
      $display(" UNFAIRNESS SPREAD    : %0.2f ranks (%0.1f %% of NCH)",
               rmax-rmin, 100.0*(rmax-rmin)/NCH);
      $display("   flat  -> arbitration is fair even on exact ties");
      $display("   sloped-> systematic bias = P1 'artifacts and delay by");
      $display("            unfair arbitration' ([S1] slide 10), and the");
      $display("            spread converts directly into timestamp error.");
    end
  endtask

  //--------------------------------------------------------------------
  // PHASE 3 report
  //--------------------------------------------------------------------
  task report3;
    integer n, g, a, l, d;
    real thr, lossp;
    begin
      g=0; a=0; l=0; d=0;
      for (n = 0; n < NCH; n = n + 1) begin
        g=g+gen_cnt[n]; a=a+acc_cnt[n]; l=l+lost_cnt[n]; d=d+done_cnt[n];
      end
      thr   = (d * 1.0e3) / 200000.0;
      lossp = (g > 0) ? (100.0*l)/g : 0.0;
      p3_pass = (err_cnt == 0) && ((a - d) <= NCH-1);
      $display(" generated/accepted/lost/done : %0d / %0d / %0d / %0d", g, a, l, d);
      $display(" loss %0.3f %%   throughput %0.3f Meps", lossp, thr);
      if (d > 0)
        $display(" latency mean %0.3f ns (%0.3f ns/event-bit)  min %0.3f  WORST %0.3f",
                 lat_sum/d, (lat_sum/d)/DEPTH, lat_min, lat_max);
      $display(" root cap/fwd %0d/%0d   in-flight %0d / %0d   ERRORS %0d",
               capR, fwdR, a-d, NCH-1, err_cnt);
      if (a - d > NCH-1)
        $display(" *** SILENT LOSS: ~%0d acknowledged but never emitted", (a-d)-(NCH-1));
    end
  endtask

  //--------------------------------------------------------------------
  // event sources
  //--------------------------------------------------------------------
  genvar gi;
  generate
    for (gi = 0; gi < NCH; gi = gi + 1) begin : g_src
      integer seed_i;
      real    iat, u;
      initial begin
        seed_i = 32'h5EED0000 + gi*7919 + 1;
        forever begin
          wait (gen_en === 1'b1);
          u   = $dist_uniform(seed_i, 1, 1000000) / 1000000.0;
          iat = -$ln(u) / lambda_ch;
          #(iat);
          if (gen_en) fire_event(gi);
        end
      end
      initial begin
        forever begin
          @(posedge ev_ack[gi]);
          ev_req[gi] = 1'b0;
          @(negedge ev_ack[gi]);
          busy[gi]   = 1'b0;
        end
      end
    end
  endgenerate

  //--------------------------------------------------------------------
  // sink
  //--------------------------------------------------------------------
  always @(posedge out_req) begin : b_meas
    integer c;
    real    lat;
    c = out_addr;
    if (bq_wr[c] == bq_rd[c]) begin
      if (err_cnt < ERR_PRINT_MAX)
        $display("[%0t] ERROR: output claims channel %0d with no pending event",
                 $realtime, c);
      else if (err_cnt == ERR_PRINT_MAX)
        $display("[%0t] ... further address errors suppressed", $realtime);
      err_cnt = err_cnt + 1;
    end else begin
      lat = $realtime - bq[c*QD + (bq_rd[c] % QD)];
      bq_rd[c]    = bq_rd[c] + 1;
      done_cnt[c] = done_cnt[c] + 1;
      total_done  = total_done + 1;
      lat_sum     = lat_sum + lat;
      lat_sq      = lat_sq  + lat*lat;
      clat_sum[c] = clat_sum[c] + lat;
      clat_sq [c] = clat_sq [c] + lat*lat;
      if (lat > lat_max) lat_max = lat;
      if (lat < lat_min) lat_min = lat;

      // PHASE 5 : rank bookkeeping
      if (burst_mode) begin
        rank_sum[c] = rank_sum[c] + burst_rank;
        blat_sum[c] = blat_sum[c] + lat;
        if (burst_rank > rank_max[c]) rank_max[c] = burst_rank;
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
