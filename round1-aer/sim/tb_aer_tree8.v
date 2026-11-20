`timescale 1ns/1ps
//======================================================================
// tb_aer_tree8 : testbench for the 8-event AER tree
//
//   Three phases, run in order:
//
//     PHASE 1  DIRECTED   one channel at a time.
//                         Checks the address encoding of all 8 inputs.
//                         Must pass before anything else means anything.
//
//     PHASE 2  BURST      all 8 channels raised simultaneously.
//                         This is the stress test for the back-to-back
//                         grant race flagged in aer_node.v: after a
//                         child releases, the mutex must pass through a
//                         both-grants-low window wide enough for cin to
//                         return to 0. If it does not, events are lost
//                         silently. PASS = exactly 8 outputs, one per
//                         channel, no duplicates.
//
//     PHASE 3  POISSON    statistics under load.
//                         Stimulus model follows the literature so the
//                         numbers are comparable:
//                           - Poisson arrivals   [Boahen TCAS-II 2000 §III;
//                             Ryu CVPR19 s13; Samsung ICRA17 s30;
//                             Gorni & Deptuch arXiv:2511.03705]
//                           - per-source ONE-SLOT gating: a source holds
//                             at most one pending request; an event
//                             arriving while the source is busy is LOST
//                             at the source, not in a central queue
//                             [Gorni & Deptuch §3]
//
//   Reported (the T4 metric set):
//     admitted throughput [MEvent/s], mean and WORST-CASE latency [ns],
//     loss probability [%], ns per event-bit, per-channel fairness.
//     Worst case is the number that matters - the iniVation White Paper
//     (May 2020) criticises the literature for publishing only
//     small-stimulus averages.
//
//   Vivado / XSim:
//     add rtl/*.v and sim/tb_aer_tree8.v, set tb_aer_tree8 as top,
//     run behavioural simulation. Defaults are simulation-safe
//     (behavioural C-element, mutex and delay). No defines needed.
//     Optional: +RATE_MEPS=<f>  +SIM_US=<f>
//======================================================================
module tb_aer_tree8;

  localparam integer NCH     = 8;
  localparam integer QD      = 16;    // per-channel birth-time queue depth
  localparam integer REQ_DLY = 4;
  localparam real    SINK_NS = 0.5;   // downstream response time

  // Cap on repeated error printing. A stuck node can emit an address
  // every couple of ns for the whole run; left uncapped that produced
  // 104k identical lines, which pushed the BUILD banner and the PHASE
  // 1/2 verdicts out of the Vivado console scrollback and made the log
  // unreadable. The count is still exact, only the printing stops.
  localparam integer ERR_PRINT_MAX = 20;

  // C-element output delay [ns]. MUST be > 0 and MUST stay below the
  // mutex both-grants-low window (2*M_TPD structural / 2.5*M_TPD
  // behavioural). Sweep it upward until PHASE 2 breaks: that boundary is
  // the design's real timing margin. See c_element.v.
  parameter real C_TPD = 0.020;

  // mutex base gate delay [ns]. The both-grants-low handover window is
  // 2*M_TPD (structural) / 2.5*M_TPD (behavioural) and must stay wider
  // than C_TPD. See mutex.v and the handover-window constraint.
  parameter real M_TPD = 0.020;

  //--------------------------------------------------------------------
  // DUT
  //--------------------------------------------------------------------
  reg         rst_n;
  reg  [7:0]  ev_req;
  wire [7:0]  ev_ack;
  wire        out_req;
  wire [2:0]  out_addr;
  reg         out_ack;

  aer_tree8 #(.REQ_DLY(REQ_DLY), .C_TPD(C_TPD), .M_TPD(M_TPD)) dut (
    .rst_n    (rst_n),
    .ev_req   (ev_req),
    .ev_ack   (ev_ack),
    .out_req  (out_req),
    .out_addr (out_addr),
    .out_ack  (out_ack)
  );

  //--------------------------------------------------------------------
  // knobs / state
  //--------------------------------------------------------------------
  real    rate_meps;      // TOTAL offered rate across all 8 channels
  real    lambda_ch;      // per-channel rate [events / ns]
  real    sim_ns;
  reg     gen_en;         // enables the Poisson generators (phase 3)

  integer gen_cnt  [0:NCH-1];
  integer acc_cnt  [0:NCH-1];   // accepted (not dropped at the source)
  integer lost_cnt [0:NCH-1];
  integer done_cnt [0:NCH-1];
  reg     busy     [0:NCH-1];

  real    bq       [0:NCH*QD-1];   // birth-time queue (flattened)
  integer bq_wr    [0:NCH-1];
  integer bq_rd    [0:NCH-1];

  real    lat_sum, lat_max, lat_min;
  integer total_done, total_gen, total_acc, total_lost;
  integer err_cnt;
  real    t_start;

  integer p2_base  [0:NCH-1];   // done_cnt snapshot taken before PHASE 2

  // phase verdicts, restated at the very end.
  //   Vivado auto-runs 1000 ns at launch and only then echoes "run -all",
  //   so PHASE 1/2 output can scroll ABOVE the run -all line and look
  //   missing if you only read the tail of the console. The verdict
  //   block printed last is the one to trust.
  reg p1_pass, p2_pass, p3_pass;
  reg p0_pass;
  reg p1_stuck;
  reg sweep_dead;

  // both-grants-low handover window of whichever mutex model is compiled
  // in. the handover-window constraint requires gnt_window > C_TPD.
`ifdef AER_STRUCT_MUTEX
  localparam real gnt_window = 2.0 * M_TPD;
`else
  localparam real gnt_window = 2.5 * M_TPD;
`endif

  //--------------------------------------------------------------------
  // PER-NODE PROBES
  //
  //   An event dies somewhere between "a node latched it" and "the node
  //   handed it to its parent". Counting both edges at every one of the
  //   7 nodes localises the fault exactly, instead of inferring it from
  //   which source channel went missing.
  //
  //     cap = posedge cin    -> node latched an address into its FF
  //     fwd = posedge r_req  -> node presented it to its parent
  //
  //   Healthy 8-event burst:
  //     lvl0  2/2 each        (8 captures, 8 forwards)
  //     lvl1  4/4 each
  //     root  8/8
  //   cap > fwd  at a node  -> that node latched and then dropped it
  //   cap short of expected -> the loss is BELOW it, not here
  //--------------------------------------------------------------------
  integer cap0 [0:3], fwd0 [0:3];
  integer cap1 [0:1], fwd1 [0:1];
  integer capR, fwdR;

  genvar gm;
  generate
    for (gm = 0; gm < 4; gm = gm + 1) begin : g_mon0
      always @(posedge dut.g_lvl0[gm].u_n.cin)   cap0[gm] = cap0[gm] + 1;
      always @(posedge dut.g_lvl0[gm].u_n.r_req) fwd0[gm] = fwd0[gm] + 1;
    end
    for (gm = 0; gm < 2; gm = gm + 1) begin : g_mon1
      always @(posedge dut.g_lvl1[gm].u_n.cin)   cap1[gm] = cap1[gm] + 1;
      always @(posedge dut.g_lvl1[gm].u_n.r_req) fwd1[gm] = fwd1[gm] + 1;
    end
  endgenerate
  always @(posedge dut.u_root.cin)   capR = capR + 1;
  always @(posedge dut.u_root.r_req) fwdR = fwdR + 1;

  task clear_probes;
    integer n;
    begin
      for (n = 0; n < 4; n = n + 1) begin cap0[n]=0; fwd0[n]=0; end
      for (n = 0; n < 2; n = n + 1) begin cap1[n]=0; fwd1[n]=0; end
      capR = 0; fwdR = 0;
    end
  endtask

  //--------------------------------------------------------------------
  // PHASE 4 helper : one point on the capacity curve.
  //
  //   A single offered rate says only "the tree kept up". The capacity
  //   is where throughput stops tracking the offered rate and loss takes
  //   off. That knee is the number to compare against R22's M/D/1 +
  //   per-source one-slot gating prediction, and its SHAPE is what
  //   [S1] slide 13 plots (timestamp error vs event rate).
  //
  //   Each point starts from a settled, re-aligned state so it cannot
  //   inherit the previous point's in-flight events or queue offsets.
  //--------------------------------------------------------------------
  task sweep_point;
    input real r_meps;
    input real dur_ns;
    real    thr, lossp, mean_lat, spread;
    integer n, g, a, l, d, inflight, dmin, dmax;
    begin
     if (!sweep_dead) begin
      gen_en = 1'b0;
      #2000;                                   // settle
      for (n = 0; n < NCH; n = n + 1) begin
        gen_cnt[n]=0; acc_cnt[n]=0; lost_cnt[n]=0; done_cnt[n]=0;
        bq_wr[n]=0;   bq_rd[n]=0;
      end
      lat_sum=0.0; lat_max=0.0; lat_min=1.0e30; total_done=0;
      lambda_ch = (r_meps * 1.0e-3) / NCH;

      gen_en = 1'b1;
      #(dur_ns);
      gen_en = 1'b0;
      #2000;                                   // drain before counting

      g=0; a=0; l=0; d=0;
      for (n = 0; n < NCH; n = n + 1) begin
        g = g + gen_cnt[n];  a = a + acc_cnt[n];
        l = l + lost_cnt[n]; d = d + done_cnt[n];
      end
      inflight = a - d;
      // events completed during the drain were offered inside the
      // window, so the window length is the right denominator
      thr      = (d * 1.0e3) / dur_ns;
      lossp    = (g > 0) ? (100.0*l)/g : 0.0;
      mean_lat = (d > 0) ? lat_sum/d : 0.0;

      //----------------------------------------------------------------
      // FAIRNESS. Below saturation this is just Poisson arrival noise
      // (sigma/mean = 1/sqrt(events per channel)), so it says nothing.
      // AT SATURATION it is the real measurement: every mutex starts
      // from the same LFSR seed (8'hA5, so tie_pick=1 and g2 always wins
      // a simultaneous tie), and if that biases service, the channels
      // that keep losing ties will show a systematically lower count.
      // That IS the "unfair arbitration -> artifacts and delay" of
      // Ryu, CVPR19 workshop, slide 10.
      //----------------------------------------------------------------
      dmin = done_cnt[0]; dmax = done_cnt[0];
      for (n = 1; n < NCH; n = n + 1) begin
        if (done_cnt[n] < dmin) dmin = done_cnt[n];
        if (done_cnt[n] > dmax) dmax = done_cnt[n];
      end
      spread = (d > 0) ? (100.0*(dmax-dmin)) / (d*1.0/NCH) : 0.0;

      $display("  %7.1f | %9.2f | %7.3f | %8.3f | %10.3f | %6.2f | %s",
               r_meps, thr, lossp, mean_lat, lat_max, spread,
               (inflight <= NCH-1) ? "ok" : "*** SILENT LOSS ***");
      // a point that delivered nothing means the DUT is wedged; the
      // remaining points would burn minutes of wall clock for no data
      if (d == 0) begin
        sweep_dead = 1'b1;
        $display("  -> zero completions: DUT wedged, skipping remaining points");
      end
     end
    end
  endtask

  // per-channel breakdown, meaningful only once loss is non-trivial
  task dump_fairness;
    integer n;
    begin
      $write("   per-channel done :");
      for (n = 0; n < NCH; n = n + 1) $write(" %6d", done_cnt[n]);
      $display("");
      $write("   per-channel lost :");
      for (n = 0; n < NCH; n = n + 1) $write(" %6d", lost_cnt[n]);
      $display("");
      $write("   loss rate [%%]    :");
      for (n = 0; n < NCH; n = n + 1)
        $write(" %6.2f", (gen_cnt[n]>0) ? (100.0*lost_cnt[n])/gen_cnt[n] : 0.0);
      $display("");
    end
  endtask

  task dump_probes;
    begin
      $display(" node cap/fwd (latched / handed to parent)");
      $display("   lvl0 : %0d/%0d  %0d/%0d  %0d/%0d  %0d/%0d   [expect 2/2 each]",
               cap0[0],fwd0[0], cap0[1],fwd0[1], cap0[2],fwd0[2], cap0[3],fwd0[3]);
      $display("   lvl1 : %0d/%0d  %0d/%0d                     [expect 4/4 each]",
               cap1[0],fwd1[0], cap1[1],fwd1[1]);
      $display("   root : %0d/%0d                             [expect 8/8]",
               capR, fwdR);
    end
  endtask

  integer k;

  //--------------------------------------------------------------------
  // helper : inject one event on channel ch
  //--------------------------------------------------------------------
  task fire_event(input integer ch);
    begin
      gen_cnt[ch] = gen_cnt[ch] + 1;
      if (busy[ch]) begin
        lost_cnt[ch] = lost_cnt[ch] + 1;      // one-slot gating at source
      end else begin
        busy[ch]                     = 1'b1;
        acc_cnt[ch]                  = acc_cnt[ch] + 1;
        bq[ch*QD + (bq_wr[ch] % QD)] = $realtime;
        bq_wr[ch]                    = bq_wr[ch] + 1;
        ev_req[ch]                   = 1'b1;
      end
    end
  endtask

  task parse_plusargs;
    begin
      if ($value$plusargs("RATE_MEPS=%f", rate_meps)) ;
      if ($value$plusargs("SIM_US=%f", sim_ns)) sim_ns = sim_ns * 1000.0;
    end
  endtask

  //--------------------------------------------------------------------
  // main
  //--------------------------------------------------------------------
  initial begin : main
    integer ch;
    integer before;

    rate_meps = 50.0;
    sim_ns    = 200000.0;                 // 200 us
    parse_plusargs;
    lambda_ch = (rate_meps * 1.0e-3) / NCH;   // MEvent/s -> events/ns per channel

    for (k = 0; k < NCH; k = k + 1) begin
      gen_cnt[k]=0; acc_cnt[k]=0; lost_cnt[k]=0; done_cnt[k]=0;
      busy[k]=1'b0; bq_wr[k]=0; bq_rd[k]=0;
    end
    lat_sum=0.0; lat_max=0.0; lat_min=1.0e30;
    total_done=0; total_gen=0; total_acc=0; total_lost=0; err_cnt=0;
    p0_pass=1'b0; p1_pass=1'b0; p2_pass=1'b0; p3_pass=1'b0;
    p1_stuck=1'b0; sweep_dead=1'b0;

    //----------------------------------------------------------------
    // BUILD BANNER - the log must record WHICH circuit was simulated.
    // The behavioural and structural models are different circuits;
    // only the structural one is what gets synthesised.
    //----------------------------------------------------------------
    $display("");
    $display("===== BUILD =====");
`ifdef AER_STRUCT_C
    $display(" c_element   : STRUCTURAL  (Earle latch, 3 AND + 1 OR, real comb loop)");
`else
    $display(" c_element   : behavioural (inferred latch)");
`endif
`ifdef AER_STRUCT_MUTEX
    $display(" mutex       : STRUCTURAL  (cross-coupled NAND), window = %0.3f ns", gnt_window);
    $display("               tie-break by %0.3f ns device-mismatch skew (r1 wins)",
             dut.g_lvl0[0].u_n.u_mutex.MISMATCH_NS);
`else
    $display(" mutex       : behavioural (LFSR tie-break),     window = %0.3f ns", gnt_window);
`endif
`ifdef AER_STRUCT_DLY
    $display(" delay_chain : STRUCTURAL  (buffer chain)");
`else
    $display(" delay_chain : behavioural (lumped inertial)");
`endif
    $display(" C_TPD %0.3f ns / M_TPD %0.3f ns / REQ_DLY %0d / SINK %0.2f ns",
             C_TPD, M_TPD, REQ_DLY, SINK_NS);
    // §4.2 margin. Unequal-length string literals in a ternary get
    // zero-padded in Verilog, so use plain if/else instead.
    if (gnt_window > C_TPD)
      $display(" margin check: window %0.3f ns > C-element %0.3f ns  -> ok",
               gnt_window, C_TPD);
    else
      $display(" margin check: window %0.3f ns <= C-element %0.3f ns -> TOO TIGHT, expect PHASE 2 to fail",
               gnt_window, C_TPD);
    $display("=================");

    gen_en  = 1'b0;
    ev_req  = 8'h00;
    out_ack = 1'b0;
    rst_n   = 1'b0;
    #20 rst_n = 1'b1;
    #10;

    //================================================================
    // PHASE 1 : directed, one channel at a time
    //================================================================
    //================================================================
    // PHASE 0 : IDLE - nothing has been fired, nothing may move.
    //
    //   Separates "the circuit free-runs on its own" from "the circuit
    //   mishandles a request". A structural build with a real
    //   combinational loop can oscillate straight out of reset, and if
    //   it does, every later phase is measuring noise rather than
    //   protocol behaviour.
    //================================================================
    $display("");
    $display("---------- PHASE 0 : IDLE ----------");
    clear_probes;
    #500;
    p0_pass = (capR == 0) && (fwdR == 0);
    if (p0_pass) $display("PHASE 0 PASS : idle for 500 ns, no spurious activity");
    else begin
      $display("PHASE 0 FAIL : DUT is SELF-OSCILLATING with no stimulus");
      $display("   ev_req=%b ev_ack=%b out_req=%b out_addr=%b out_ack=%b",
               ev_req, ev_ack, out_req, out_addr, out_ack);
      dump_probes;
      $display("   -> a node is cycling on its own. Later phases cannot");
      $display("      mean anything until this is fixed.");
    end

    $display("\n---------- PHASE 1 : DIRECTED ----------");
    p1_stuck = 1'b0;
    for (ch = 0; ch < NCH; ch = ch + 1) begin
      before = total_done;
      fire_event(ch);
      // bounded wait: an un-acked channel would otherwise hang here
      // forever with no message at all
      fork : wait_p1
        begin wait (total_done == before + 1); disable wait_p1; end
        begin #1000;
              $display("PHASE 1 STUCK : channel %0d fired but produced no output in 1 us",
                       ch);
              $display("   ev_req=%b ev_ack=%b out_req=%b out_addr=%b out_ack=%b",
                       ev_req, ev_ack, out_req, out_addr, out_ack);
              dump_probes;
              p1_stuck = 1'b1;
              disable wait_p1;
        end
      join
      if (p1_stuck) ch = NCH;          // stop after the first stuck channel
      #20;
    end
    p1_pass = (err_cnt == 0) && !p1_stuck;
    if (p1_pass) $display("PHASE 1 PASS : all 8 addresses encoded correctly");
    else         $display("PHASE 1 FAIL : %0d error(s), stuck=%0d", err_cnt, p1_stuck);
    #100;

    //================================================================
    // PHASE 2 : burst, all 8 at once  (grant-race stress)
    //================================================================
    $display("\n---------- PHASE 2 : BURST ----------");
    before  = total_done;
    for (ch = 0; ch < NCH; ch = ch + 1) p2_base[ch] = done_cnt[ch];
    clear_probes;
    for (ch = 0; ch < NCH; ch = ch + 1) fire_event(ch);

    fork : wait_burst
      begin wait (total_done == before + 8); disable wait_burst; end
      begin #20000;
            $display("PHASE 2 FAIL : only %0d/8 events came out (event loss or deadlock)",
                     total_done - before);
            err_cnt = err_cnt + 1;
            disable wait_burst;
      end
    join
    p2_pass = (total_done == before + 8);
    if (p2_pass)
      $display("PHASE 2 PASS : all 8 contending events delivered");

    //----------------------------------------------------------------
    // WHERE was it lost?  The address of a missing channel says which
    // tree level dropped it:
    //   odd-only  {1,3,5,7}  -> the second child of every level-0 node
    //                           = level-0 grant handover
    //   {2,3} / {6,7} style  -> level-1 handover
    //   {4,5,6,7}            -> root handover
    // A channel that is still busy did NOT get its handshake closed,
    // which is a different fault from an event that was acknowledged
    // and then dropped.
    //----------------------------------------------------------------
    $write(" per-channel delivered :");
    for (ch = 0; ch < NCH; ch = ch + 1) $write(" %0d", done_cnt[ch]-p2_base[ch]);
    $display("");
    $write(" MISSING channels      :");
    for (ch = 0; ch < NCH; ch = ch + 1)
      if (done_cnt[ch]-p2_base[ch] == 0) $write(" %0d", ch);
    $display("");
    $write(" still busy (no ack)   :");
    for (ch = 0; ch < NCH; ch = ch + 1) if (busy[ch]) $write(" %0d", ch);
    $display("");
    dump_probes;
    #200;

    //================================================================
    // PHASE 3 : Poisson load
    //================================================================
    $display("\n---------- PHASE 3 : POISSON ----------");

    //----------------------------------------------------------------
    // DRAIN + QUEUE REALIGNMENT - required.
    //
    //   bq_wr/bq_rd are NOT part of the counter reset below by accident;
    //   they must be reset explicitly. An event lost in PHASE 2 leaves
    //   bq_wr[ch] != bq_rd[ch] forever, so every later latency on that
    //   channel is measured against a birth time that is N events stale.
    //   That silently inflates mean and worst latency by the age of the
    //   leftover entry (which is why a broken PHASE 2 used to produce a
    //   ~20 us "worst case" that was really just the PHASE 2 timeout).
    //----------------------------------------------------------------
    #1000;
    for (k = 0; k < NCH; k = k + 1)
      if (bq_wr[k] != bq_rd[k])
        $display("[tb] ch %0d : %0d un-delivered event(s) left over from PHASE 2 (queue realigned)",
                 k, bq_wr[k]-bq_rd[k]);

    $display("[tb] offered rate = %0.2f MEvent/s total, sim = %0.1f us",
             rate_meps, sim_ns/1000.0);
    t_start = $realtime;
    for (k = 0; k < NCH; k = k + 1) begin
      gen_cnt[k]=0; acc_cnt[k]=0; lost_cnt[k]=0; done_cnt[k]=0;
      bq_wr[k]=0;   bq_rd[k]=0;
    end
    lat_sum=0.0; lat_max=0.0; lat_min=1.0e30; total_done=0;
    err_cnt = 0;                 // PHASE 1/2 verdicts are already printed
    clear_probes;

    gen_en = 1'b1;
    #(sim_ns);
    gen_en = 1'b0;

    report;

    //================================================================
    // PHASE 4 : capacity sweep  (push until it saturates)
    //================================================================
    $display("\n---------- PHASE 4 : CAPACITY SWEEP ----------");
    $display("  offered |  thruput  |   loss  |   mean   |   worst    | spread | in-flight");
    $display("   [Meps] |   [Meps]  |    [%%]  |   [ns]   |    [ns]    |   [%%]  |");
    $display("  --------+-----------+---------+----------+------------+--------+-----------");
    // Window lengths are scaled so every point collects >=10k events;
    // at a flat 50 us the 50 Meps point only saw 2.5k, i.e. +-2% Poisson
    // noise, which is why it read 48.96 while 100 Meps read 101.78.
    // Extra points at 350/450/550 resolve the knee.
    sweep_point(  50.0, 200000.0);
    sweep_point( 100.0, 150000.0);
    sweep_point( 200.0, 100000.0);
    sweep_point( 300.0,  80000.0);
    sweep_point( 350.0,  70000.0);
    sweep_point( 400.0,  60000.0);
    sweep_point( 450.0,  60000.0);
    sweep_point( 500.0,  50000.0);
    sweep_point( 550.0,  50000.0);
    sweep_point( 600.0,  50000.0);
    sweep_point( 700.0,  50000.0);
    sweep_point( 850.0,  50000.0);
    sweep_point(1000.0,  50000.0);
    $display("  knee = where thruput stops tracking offered and loss takes off.");
    $display("");
    $display("  FAIRNESS AT SATURATION (last point above, ~46%% dropped):");
    dump_fairness;
    $display("   -> loss rates must match across channels. A channel that");
    $display("      systematically loses ties would show a higher loss rate;");
    $display("      that is the 'unfair arbitration' axis ([S1] slide 10).");
    $display("  NOTE: absolute values reflect the BEHAVIOURAL delays");
    $display("        (mutex 60/10ps, C_TPD %0.3fns, REQ_DLY %0d, SINK %0.2fns),",
             C_TPD, REQ_DLY, SINK_NS);
    $display("        NOT silicon. Compare the SHAPE, not the numbers.");

    //================================================================
    // FINAL VERDICT - printed last so it cannot be missed
    //================================================================
    $display("\n================= T1 VERDICT =================");
    $display("   PHASE 0  IDLE     (no self-osc) : %s", p0_pass ? "PASS" : "FAIL");
    $display("   PHASE 1  DIRECTED (encoding)   : %s", p1_pass ? "PASS" : "FAIL");
    $display("   PHASE 2  BURST    (grant race) : %s", p2_pass ? "PASS" : "FAIL");
    $display("   PHASE 3  POISSON  (no silent loss, no addr err) : %s",
             p3_pass ? "PASS" : "FAIL");
    $display("   ------------------------------------------");
    $display("   OVERALL : %s",
             (p0_pass && p1_pass && p2_pass && p3_pass) ? "PASS" : "FAIL");
    $display("==============================================");

    $finish;
  end

  //--------------------------------------------------------------------
  // event sources : Poisson arrivals (phase 3 only)
  //--------------------------------------------------------------------
  genvar gi;
  generate
    for (gi = 0; gi < NCH; gi = gi + 1) begin : g_src

      integer seed_i;
      real    iat;
      real    u;

      initial begin
        seed_i = 32'h5EED0000 + gi*7919 + 1;
        forever begin
          wait (gen_en === 1'b1);
          u   = $dist_uniform(seed_i, 1, 1000000) / 1000000.0;
          iat = -$ln(u) / lambda_ch;              // exponential inter-arrival
          #(iat);
          if (gen_en) fire_event(gi);
        end
      end

      // 4-phase completion at the source side (always active)
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
  // sink : latency capture, address check, 4-phase acknowledge
  //--------------------------------------------------------------------
  always @(posedge out_req) begin : b_meas
    integer ch;
    real    lat;
    ch = out_addr;
    if (bq_wr[ch] == bq_rd[ch]) begin
      if (err_cnt < ERR_PRINT_MAX)
        $display("[%0t] ERROR: output claims channel %0d but that channel has no pending event",
                 $realtime, ch);
      else if (err_cnt == ERR_PRINT_MAX)
        $display("[%0t] ... further address errors suppressed, still counted in ERRORS",
                 $realtime);
      err_cnt = err_cnt + 1;
    end else begin
      lat = $realtime - bq[ch*QD + (bq_rd[ch] % QD)];
      bq_rd[ch]    = bq_rd[ch] + 1;
      done_cnt[ch] = done_cnt[ch] + 1;
      total_done   = total_done + 1;
      lat_sum      = lat_sum + lat;
      if (lat > lat_max) lat_max = lat;
      if (lat < lat_min) lat_min = lat;
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

  //--------------------------------------------------------------------
  // report
  //--------------------------------------------------------------------
  task report;
    real elapsed_ns, thr_meps, loss_pct;
    begin
      elapsed_ns = $realtime - t_start;
      total_gen = 0; total_acc = 0; total_lost = 0;
      for (k = 0; k < NCH; k = k + 1) begin
        total_gen  = total_gen  + gen_cnt[k];
        total_acc  = total_acc  + acc_cnt[k];
        total_lost = total_lost + lost_cnt[k];
      end
      thr_meps = (total_done * 1.0e3) / elapsed_ns;   // events/ns -> MEvent/s
      loss_pct = (total_gen > 0) ? (100.0*total_lost)/total_gen : 0.0;

      $display("");
      $display("==================== T1 RESULT ====================");
      $display(" C_TPD / REQ_DLY   : %0.3f ns / %0d", C_TPD, REQ_DLY);
      $display(" offered rate      : %0.2f MEvent/s", rate_meps);
      $display(" elapsed           : %0.1f us", elapsed_ns/1000.0);
      $display(" generated         : %0d", total_gen);
      $display(" accepted / lost   : %0d / %0d", total_acc, total_lost);
      $display(" LOSS PROBABILITY  : %0.3f %%", loss_pct);
      $display(" completed         : %0d   (accepted-completed = %0d)",
               total_done, total_acc - total_done);

      //--------------------------------------------------------------
      // IN-FLIGHT SANITY - this is the silent-loss detector.
      //
      //   The tree holds at most one event per node, so at the instant
      //   the generators stop, (accepted - completed) cannot exceed
      //   NCH-1 = 7. Anything above that was acknowledged to its source
      //   and then dropped inside the tree, i.e. SILENT LOSS.
      //
      //   It also invalidates the latency figures below: each lost
      //   event leaves that channel's birth-time queue one slot behind
      //   for the rest of the run, so every later latency on that
      //   channel is measured against a stale birth time.
      //--------------------------------------------------------------
      p3_pass = (err_cnt == 0) && ((total_acc - total_done) <= NCH-1);
      if (total_acc - total_done > NCH-1) begin
        $display(" *** SILENT LOSS   : ~%0d event(s) acknowledged but never emitted",
                 (total_acc - total_done) - (NCH-1));
        $display("     -> LATENCY FIGURES BELOW ARE INVALID (stale birth times)");
      end else
        $display(" in-flight         : %0d / %0d slots - OK, latency figures valid",
                 total_acc - total_done, NCH-1);

      $display(" THROUGHPUT        : %0.3f MEvent/s", thr_meps);
      if (total_done > 0) begin
        $display(" latency mean      : %0.3f ns  (%0.3f ns/event-bit)",
                 lat_sum/total_done, (lat_sum/total_done)/3.0);
        $display(" latency min       : %0.3f ns", lat_min);
        $display(" LATENCY WORST     : %0.3f ns  <-- report this one", lat_max);
      end
      $display(" per-channel done  : %0d %0d %0d %0d %0d %0d %0d %0d",
               done_cnt[0],done_cnt[1],done_cnt[2],done_cnt[3],
               done_cnt[4],done_cnt[5],done_cnt[6],done_cnt[7]);
      $display("   (fairness: these should be within a few %% of each");
      $display("    other. A systematic skew IS the 'unfair arbitration'");
      $display("    that Ryu CVPR19 s10 calls out as the root cause of");
      $display("    artifacts and delay.)");
      $display(" ERRORS            : %0d", err_cnt);
      $display("---------------------------------------------------");
      dump_probes;
      $display("   (steady state: every node's cap should equal its fwd,");
      $display("    and each level's total should equal the level below.");
      $display("    A cap>fwd gap names the node that drops events.)");
      $display("===================================================");
    end
  endtask

  //--------------------------------------------------------------------
  // watchdog : C-element loops can deadlock
  //--------------------------------------------------------------------
  real last_activity;
  always @(posedge out_req) last_activity = $realtime;
  initial begin
    last_activity = 0.0;
    forever begin
      #10000;
      if (gen_en && ($realtime - last_activity > 50000.0)) begin
        $display("[tb] *** DEADLOCK: no output event for 50 us ***");
        report;
        $finish;
      end
    end
  end

`ifdef AER_DUMP
  initial begin
    $dumpfile("aer_tree8.vcd");
    $dumpvars(0, tb_aer_tree8);
  end
`endif

endmodule
