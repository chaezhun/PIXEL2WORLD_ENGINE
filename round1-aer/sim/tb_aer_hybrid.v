`timescale 1ns/1ps
//======================================================================
// tb_aer_hybrid : P1 bench, and the B1/B2 baseline bench, in one file
//
//   TH_ROWS > NROW disables the snapshot mechanism, so aer_hybrid
//   degenerates to plain row/column burst readout. Running the same
//   bench twice - once with TH_ROWS=9 (baseline) and once with
//   TH_ROWS=4 (proposal) - compares them under IDENTICAL stimulus and
//   identical measurement code. Nothing else changes.
//
//----------------------------------------------------------------------
// SPATIAL CLUSTERING (the T4 gap found in characterisation)
//
//   Every measurement so far used per-pixel independent Poisson, which
//   has ZERO spatial correlation. Under that traffic the burst length
//   measured 1.14 - and 1/(1-1/NROW) = 1.143 is exactly what independent
//   rows give by coincidence, so the real bursting contribution was nil
//   until the array congested.
//
//   Real DVS events do not look like that. They land on edges and on
//   moving objects, so several pixels of the same row (or column) fire
//   together even at low total rate. [S2]'s 32 -> 12 -> 8 bit/event is
//   measured on scenes with that structure; iid Poisson destroys it.
//
//   CLUSTER selects the spatial model. Total offered rate is held
//   constant across modes, so only the SHAPE of the traffic changes:
//     0 = iid Poisson         per-pixel, no correlation (previous runs)
//     1 = horizontal segment  CL_SIZE pixels along one row
//                             -> best case for row/column burst readout
//     2 = vertical segment    CL_SIZE pixels down one column
//                             -> worst case: every event needs a new row
//     3 = square block        sqrt(CL_SIZE) x sqrt(CL_SIZE)
//
//   Modes 1 and 2 are the same edge at two orientations. If they differ,
//   burst compression is ORIENTATION DEPENDENT - which matters, because
//   a real scene contains every orientation.
//======================================================================
module tb_aer_hybrid;

  parameter integer ROW_D    = 3;
  parameter integer COL_D    = 3;
  parameter integer TH_ROWS  = 4;      // > NROW disables snapshots (= B1/B2)
  parameter integer CLUSTER  = 0;      // 0 iid / 1 horiz / 2 vert / 3 block
  parameter integer CL_SIZE  = 4;
  parameter integer NBURST   = 200;

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

  //--------------------------------------------------------------------
  reg            rst_n;
  reg  [NPX-1:0] px_req;
  wire [NPX-1:0] px_ack;
  wire           out_req;
  wire [AW-1:0]  out_addr;
  reg            out_ack;
  wire           hold_pulse, hold_mode;

  aer_hybrid #(
    .ROW_D(ROW_D), .COL_D(COL_D), .TH_ROWS(TH_ROWS),
    .REQ_DLY(REQ_DLY), .C_TPD(C_TPD), .M_TPD(M_TPD)
  ) dut (
    .rst_n(rst_n), .px_req(px_req), .px_ack(px_ack),
    .out_req(out_req), .out_addr(out_addr), .out_ack(out_ack),
    .hold_pulse(hold_pulse), .hold_mode(hold_mode)
  );

  //--------------------------------------------------------------------
  real    rate_meps, lambda_cl;      // cluster arrival rate [1/ns]
  reg     gen_en;

  integer gen_cnt [0:NPX-1];
  integer acc_cnt [0:NPX-1];
  integer lost_cnt[0:NPX-1];
  integer done_cnt[0:NPX-1];
  reg     busy    [0:NPX-1];

  real    bq   [0:NPX*QD-1];
  integer bq_wr[0:NPX-1];
  integer bq_rd[0:NPX-1];

  real lat_sum, lat_sq, lat_max, lat_min;
  real clat_sum[0:NPX-1], clat_sq[0:NPX-1];
  real ts_sum,  ts_sq,  ts_max;
  real cts_sum[0:NPX-1], cts_sq[0:NPX-1];

  real    t_hold;
  integer nsnap;
  always @(posedge hold_pulse) begin t_hold = $realtime; nsnap = nsnap + 1; end

  // time spent with the snapshot mechanism engaged
  real hold_t0, hold_acc;
  always @(posedge hold_mode) hold_t0  = $realtime;
  always @(negedge hold_mode) hold_acc = hold_acc + ($realtime - hold_t0);

  integer total_done, err_cnt, k;
  reg     p0_pass, p1_pass, p2_pass, p3_pass, p1_stuck;

  //--------------------------------------------------------------------
  // EMISSION CHAIN AUDIT
  //
  //   Every event passes through exactly three edges:
  //       posedge cin      the address is latched
  //       posedge px_ack   the granted pixel is acknowledged
  //       posedge out_req  the address is emitted
  //   In a correct design all three counts are equal. Any inequality
  //   names the stage that broke:
  //       n_out > n_cin    the output stage emitted twice for one latch
  //       n_cin > n_ack    an address was latched while no pixel was
  //                        selected, i.e. the arbiter input set changed
  //                        under a transaction that was already committed
  //
  //   Per-pixel counts add a second view. ORPH/PHAN compare acknowledges
  //   against COMPLETED outputs, which excludes errored ones, so they can
  //   read zero while the chain is broken - the totals above close that
  //   gap. SPUR catches an acknowledge delivered to an idle pixel.
  //--------------------------------------------------------------------
  integer ack_cnt [0:NPX-1];

  //   ORPH and PHAN both measured ZERO after the settling gate, yet the
  //   bench still reported "no pending event". That leaves exactly one
  //   possibility, which those two counters are blind to: a pixel with
  //   NOTHING PENDING gets acknowledged and its address emitted. Both
  //   counters then rise together, so ack == done and the imbalance
  //   vanishes. SPUR catches it directly - an acknowledge arriving at a
  //   pixel that is not busy.
  integer spur_ack;

  //--------------------------------------------------------------------
  // EMISSION CHAIN AUDIT
  //
  //   Every event must pass through exactly three edges:
  //       posedge cin        the address is latched
  //       posedge px_ack     the granted pixel is acknowledged
  //       posedge out_req    the address is emitted
  //   In a correct design all three counts are equal, and equal to
  //   (successful outputs + errored outputs).
  //
  //   ORPH/PHAN compare ack against done_cnt, but done_cnt is NOT
  //   incremented for an errored output, so an emission with no
  //   matching acknowledge is invisible to them. These three counters
  //   close that gap:
  //       n_out > n_cin   the output stage emitted twice for one latch
  //                       (an extra cout pulse duplicates addr_q)
  //       n_cin > n_ack   an address was latched without acknowledging
  //                       any pixel (grants were not valid at the edge)
  //--------------------------------------------------------------------
  integer n_cin, n_ack, n_out;

  //--------------------------------------------------------------------
  // BAD-LATCH PROBE
  //
  //   px_ack = row_g & col_g & cin, so a latch that acknowledges nobody
  //   must have had row_g or col_g at zero when cin asserted. Sampling
  //   the surrounding state on that edge identifies which grant was
  //   missing and in which mode, rather than inferring it from totals.
  //   Retained as a permanent assertion: it must never fire.
  //--------------------------------------------------------------------
  function integer ones;
    input [NPX-1:0] v;
    integer j;
    begin
      ones = 0;
      for (j = 0; j < NPX; j = j + 1) if (v[j]) ones = ones + 1;
    end
  endfunction

  integer n_bad, n_bad_hold, n_bad_free;
  always @(posedge dut.cin) begin
    if (!(|dut.row_g) || !(|dut.col_g)) begin
      if (n_bad < 8)
        $display("[%0t] BAD LATCH  row_g=%0d col_g=%0d | settle=%0b hold=%0b snap=%0b | cout=%0b out_ack=%0b | held=%0d px=%0d addr=%0d",
                 $realtime, (|dut.row_g), (|dut.col_g),
                 dut.settle_n, hold_mode, dut.snap,
                 dut.cout, out_ack,
                 ones(dut.held), ones(px_req), dut.addr_q);
      n_bad = n_bad + 1;
      if (hold_mode) n_bad_hold = n_bad_hold + 1;
      else           n_bad_free = n_bad_free + 1;
    end
  end
  integer n_ack2;           // cross-check: always-block ack count
  integer err_tot;          // accumulated across sweep points
  always @(posedge dut.cin) n_cin = n_cin + 1;
  always @(posedge out_req) n_out = n_out + 1;

  integer blen_sum, blen_n, cur_row, cur_len;
  task blen_reset; begin blen_sum=0; blen_n=0; cur_row=-1; cur_len=0; end endtask

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

  // fire one cluster of CL_SIZE pixels around a random centre
  task fire_cluster(input integer cr, input integer cc);
    integer j, rr, ccx, side;
    begin
      case (CLUSTER)
        1: for (j = 0; j < CL_SIZE; j = j + 1)
             fire_event(cr*NCOL + ((cc + j) % NCOL));
        2: for (j = 0; j < CL_SIZE; j = j + 1)
             fire_event((((cr + j) % NROW))*NCOL + cc);
        3: begin
             side = 2;
             while (side*side < CL_SIZE) side = side + 1;
             for (j = 0; j < CL_SIZE; j = j + 1) begin
               rr  = (cr + (j / side)) % NROW;
               ccx = (cc + (j % side)) % NCOL;
               fire_event(rr*NCOL + ccx);
             end
           end
        default: fire_event(cr*NCOL + cc);
      endcase
    end
  endtask

  task reset_counters;
    integer n;
    begin
      for (n = 0; n < NPX; n = n + 1) begin
        gen_cnt[n]=0; acc_cnt[n]=0; lost_cnt[n]=0; done_cnt[n]=0;
        bq_wr[n]=0;   bq_rd[n]=0;
        clat_sum[n]=0.0; clat_sq[n]=0.0; cts_sum[n]=0.0; cts_sq[n]=0.0;
        ack_cnt[n]=0;
      end
      lat_sum=0.0; lat_sq=0.0; lat_max=0.0; lat_min=1.0e30;
      ts_sum=0.0;  ts_sq=0.0;  ts_max=0.0;
      total_done=0; hold_acc=0.0; nsnap=0; spur_ack=0;
      n_cin=0; n_ack=0; n_out=0; n_ack2=0;
      n_bad=0; n_bad_hold=0; n_bad_free=0;
      err_tot = err_tot + err_cnt; err_cnt = 0;
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
  task calc_stats;
    input  integer mode;                // 0 = readout latency, 1 = timestamp
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
        m  = tot/d;  vt = totsq/d - m*m;  vb = 0.0; vw = 0.0;
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
    real thr, lossp, rm, rsd, rsdb, rsdw, tm, tsd, tsdb, tsdw, kbar, hpct;
    integer n, g, a, l, d, orph, phan;
    begin
      gen_en = 1'b0;
      //--------------------------------------------------------------
      // Drain fully before clearing the counters.
      //
      //   A fixed #3000 is NOT enough for the snapshot path: while a
      //   snapshot is being drained the tail of the previous point can
      //   still be in flight. Zeroing bq_wr/bq_rd underneath it makes
      //   the next output land on an empty birth queue and report a
      //   phantom "no pending event".
      //
      //: this produced ~10 spurious errors per P1
      //   run and ZERO in the baseline (which always drains inside
      //   3000 ns), while ORPH/PHAN/SPUR all stayed at 0 - because the
      //   ack and the output both land AFTER the reset, and busy[] is
      //   not part of reset_counters. The design was never at fault.
      //--------------------------------------------------------------
      wait_idle;
      #3000;
      reset_counters; blen_reset;
      // total event rate is fixed; clusters arrive CL_SIZE times slower
      lambda_cl = (r_meps * 1.0e-3) /
                  ((CLUSTER == 0) ? NPX : ((CL_SIZE > 0) ? CL_SIZE : 1));
      gen_en = 1'b1;
      #(dur_ns);
      gen_en = 1'b0;
      #3000;

      g=0; a=0; l=0; d=0;
      for (n = 0; n < NPX; n = n + 1) begin
        g=g+gen_cnt[n]; a=a+acc_cnt[n]; l=l+lost_cnt[n]; d=d+done_cnt[n];
      end
      if (cur_len > 0) begin blen_sum=blen_sum+cur_len; blen_n=blen_n+1; end
      thr   = (d * 1.0e3) / dur_ns;
      lossp = (g > 0) ? (100.0*l)/g : 0.0;
      kbar  = (blen_n > 0) ? (1.0*blen_sum)/blen_n : 1.0;
      hpct  = 100.0*hold_acc/dur_ns;
      calc_stats(0, rm, rsd, rsdb, rsdw);
      calc_stats(1, tm, tsd, tsdb, tsdw);
      orph = 0; phan = 0;
      for (n = 0; n < NPX; n = n + 1) begin
        if (ack_cnt[n] > done_cnt[n]) orph = orph + (ack_cnt[n] - done_cnt[n]);
        if (done_cnt[n] > ack_cnt[n]) phan = phan + (done_cnt[n] - ack_cnt[n]);
      end

      $display("  %7.1f | %8.2f | %6.2f | %7.3f | %8.2f | %7.3f | %5.2f | %6.1f | %6d | %6d | %6d | %4d",
               r_meps, thr, lossp, rm, lat_max, tsd,
               kbar, hpct, n_cin, n_ack, n_out, err_cnt);
      if (n_ack2 != n_ack)
        $display("      note: independent ack count %0d differs from %0d", n_ack2, n_ack);
      if (n_bad != 0)
        $display("      bad latches: %0d total (%0d during hold, %0d outside hold)",
                 n_bad, n_bad_hold, n_bad_free);
    end
  endtask

  //--------------------------------------------------------------------
  integer p, before;

  initial begin : main
    rate_meps = 50.0;
    reset_counters; blen_reset; err_cnt=0;
    p0_pass=0; p1_pass=0; p2_pass=0; p3_pass=0; p1_stuck=0;
    t_hold=0.0; hold_t0=0.0; err_tot=0;
    for (k = 0; k < NPX; k = k + 1) busy[k]=1'b0;
    gen_en=1'b0; px_req={NPX{1'b0}}; out_ack=1'b0; rst_n=1'b0;

    $display("");
    $display("===== BUILD : P1 HYBRID =====");
    $display(" array    : %0d x %0d = %0d px, %0d-bit address", NROW, NCOL, NPX, AW);
    if (TH_ROWS > NROW)
      $display(" snapshot : DISABLED (TH_ROWS=%0d > NROW=%0d) -> pure B1/B2 baseline",
               TH_ROWS, NROW);
    else
      $display(" snapshot : ENABLED, trigger at %0d of %0d active rows", TH_ROWS, NROW);
    case (CLUSTER)
      1: $display(" stimulus : HORIZONTAL cluster, %0d px along a row", CL_SIZE);
      2: $display(" stimulus : VERTICAL cluster, %0d px down a column", CL_SIZE);
      3: $display(" stimulus : BLOCK cluster, %0d px", CL_SIZE);
      default: $display(" stimulus : iid Poisson (no spatial correlation)");
    endcase
    $display("=============================");

    #20 rst_n = 1'b1;
    #10;

    // PHASE 0/1/2 sanity -------------------------------------------
    $display("\n---------- PHASE 0 : IDLE ----------");
    before = total_done; #1000;
    p0_pass = (total_done == before) && (err_cnt == 0);
    $display("PHASE 0 %s", p0_pass ? "PASS" : "FAIL");

    $display("\n---------- PHASE 1 : DIRECTED ----------");
    for (p = 0; p < NPX; p = p + 1) begin
      before = total_done;
      fire_event(p);
      fork : wp1
        begin wait (total_done == before + 1); disable wp1; end
        begin #5000; $display("PHASE 1 STUCK : px %0d", p);
              p1_stuck=1'b1; disable wp1; end
      join
      if (p1_stuck) p = NPX;
      wait_idle;
    end
    p1_pass = (err_cnt == 0) && !p1_stuck;
    $display("PHASE 1 %s : %0d px, %0d error(s)", p1_pass?"PASS":"FAIL", NPX, err_cnt);

    $display("\n---------- PHASE 2 : BURST (all %0d px) ----------", NPX);
    wait_idle; before = total_done;
    for (p = 0; p < NPX; p = p + 1) fire_event(p);
    fork : wb2
      begin wait (total_done == before + NPX); disable wb2; end
      begin #(400*NPX + 20000);
            $display("PHASE 2 FAIL : only %0d/%0d", total_done-before, NPX);
            disable wb2; end
    join
    p2_pass = (total_done == before + NPX);
    if (p2_pass) $display("PHASE 2 PASS : all %0d delivered", NPX);
    wait_idle;

    // PHASE 4 sweep --------------------------------------------------
    $display("\n---------- PHASE 4 : SWEEP ----------");
    $display("  offered |  thruput |  loss  | rd_mean | rd_worst |  ts_STD | kbar  | hold%%  |  n_cin |  n_ack |  n_out | ERR");
    $display("  --------+----------+--------+---------+----------+---------+-------+--------+--------+--------+--------+----");
    sweep_point(   1.0, 3000000.0);
    sweep_point(  10.0,  300000.0);
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
    err_tot = err_tot + err_cnt;
    p3_pass = (err_tot == 0);
    $display("  total emission errors over the sweep : %0d", err_tot);

    $display("\n============ P1 VERDICT ============");
    $display("   PHASE 0/1/2/4 : %s %s %s %s",
             p0_pass?"PASS":"FAIL", p1_pass?"PASS":"FAIL",
             p2_pass?"PASS":"FAIL", p3_pass?"PASS":"FAIL");
    $display("   OVERALL : %s",
             (p0_pass&&p1_pass&&p2_pass&&p3_pass) ? "PASS" : "FAIL");
    $display("====================================");
    $finish;
  end

  //--------------------------------------------------------------------
  // stimulus : one generator process drives whole clusters
  //--------------------------------------------------------------------
  generate
    if (CLUSTER == 0) begin : g_iid
      genvar gp;
      for (gp = 0; gp < NPX; gp = gp + 1) begin : g_px
        integer seed_i; real iat, u;
        initial begin
          seed_i = 32'h5EED0000 + gp*7919 + 1;
          forever begin
            wait (gen_en === 1'b1);
            u   = $dist_uniform(seed_i, 1, 1000000) / 1000000.0;
            iat = -$ln(u) / lambda_cl;
            #(iat);
            if (gen_en) fire_event(gp);
          end
        end
      end
    end else begin : g_clu
      integer seed_c, seed_r, seed_k;
      real    iat, u;
      initial begin
        seed_c = 32'h5EED1234; seed_r = 32'h5EED9876; seed_k = 32'h5EEDABCD;
        forever begin
          wait (gen_en === 1'b1);
          u   = $dist_uniform(seed_k, 1, 1000000) / 1000000.0;
          iat = -$ln(u) / lambda_cl;
          #(iat);
          if (gen_en)
            fire_cluster($dist_uniform(seed_r, 0, NROW-1),
                         $dist_uniform(seed_c, 0, NCOL-1));
        end
      end
    end
  endgenerate

  genvar ga;
  generate
    for (ga = 0; ga < NPX; ga = ga + 1) begin : g_ackp
      //----------------------------------------------------------------
      // Pixel four-phase completion.
      //
      //   Written as two independent always blocks rather than one
      //   forever loop. A loop that waits on posedge, then on negedge,
      //   then returns to the posedge wait can miss an edge that arrives
      //   before it re-arms. Missing an acknowledge leaves px_req
      //   asserted, so the same event is granted a second time once the
      //   snapshot is released and is emitted twice.
      //----------------------------------------------------------------
      always @(posedge px_ack[ga]) begin
        ack_cnt[ga] = ack_cnt[ga] + 1;
        if (!busy[ga]) spur_ack = spur_ack + 1;     // acked while idle
        n_ack      = n_ack + 1;
        px_req[ga] = 1'b0;
      end

      always @(negedge px_ack[ga]) busy[ga] = 1'b0;

      // independent count of the same edge, to expose a missed edge in
      // the model above rather than in the design
      always @(posedge px_ack[ga]) n_ack2 = n_ack2 + 1;
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
      lat   = $realtime - birth;
      // TIMESTAMP: snapshot instant while holding, readout instant otherwise
      tse   = hold_mode ? (t_hold - birth) : lat;
      if (tse < 0.0) tse = 0.0;
      bq_rd[a]    = bq_rd[a] + 1;
      done_cnt[a] = done_cnt[a] + 1;
      total_done  = total_done + 1;

      lat_sum = lat_sum + lat; lat_sq = lat_sq + lat*lat;
      clat_sum[a] = clat_sum[a] + lat; clat_sq[a] = clat_sq[a] + lat*lat;
      if (lat > lat_max) lat_max = lat;
      if (lat < lat_min) lat_min = lat;

      ts_sum = ts_sum + tse; ts_sq = ts_sq + tse*tse;
      cts_sum[a] = cts_sum[a] + tse; cts_sq[a] = cts_sq[a] + tse*tse;
      if (tse > ts_max) ts_max = tse;

      if ((a >> COL_D) == cur_row) cur_len = cur_len + 1;
      else begin
        if (cur_len > 0) begin blen_sum=blen_sum+cur_len; blen_n=blen_n+1; end
        cur_row = (a >> COL_D); cur_len = 1;
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
