`timescale 1ns/1ps
//======================================================================
// aer_hybrid : P1 proposal - load-adaptive snapshot timestamping over a
//              row/column burst arbiter readout.  100 % DIGITAL.
//
//----------------------------------------------------------------------
// WHERE THIS COMES FROM (all of it measured, not assumed)
//
//   This design separates two things that are usually conflated:
//       readout latency  - when the event physically leaves
//       timestamp error  - what time the event is LABELLED with
//   For an arbiter they are the same number, which is why the arbiter
//   curve in [S1] slide 13 climbs. Gen3 breaks the identity with a
//   global hold, and characterisation showed the consequence in the extreme:
//       readout rank spread 63.00 (the most rigid order possible)
//       timestamp spread     0.0000 ns
//   The artifact vanishes NOT because readout became fair but because
//   the timestamp stopped depending on readout.
//
//   But Gen3 pays for it twice:
//     - a scanner spends clocks on empty positions  -> 454 vs 538 Meps
//     - the frame period floors the timestamp error -> 34.6 ns even
//       when the array is nearly idle, where an arbiter answers in 0.5 ns
//
//   Neither penalty is intrinsic to hold-based timestamping. They come
//   from (a) scanning instead of arbitrating and (b) holding all the
//   time instead of only when it helps.
//
//----------------------------------------------------------------------
// THE PROPOSAL
//
//   Keep the arbiter readout - it skips empty positions, so no capacity
//   is lost. Add hold-based timestamping, but ONLY under load:
//
//     idle / light traffic : no snapshot. Timestamp at readout.
//                            tau_0 = 0.5 ns immediacy is preserved.
//     congested            : snapshot the pending set, stamp every event
//                            in it with the snapshot instant, drain it
//                            with the arbiter, then re-evaluate.
//
//   At light load the arbiter's timestamp error is already far below a
//   frame, so holding would only make it worse. At heavy load the
//   arbiter's readout latency is what corrupts the timestamp, and that
//   is exactly when the snapshot removes the dependence.
//
//----------------------------------------------------------------------
// DIGITAL SCOPE
//
//   Gen3's global hold lives inside the pixel as in-pixel event memory,
//   which is analogue design. It is not required here: `held` is an
//   ordinary register at the ARRAY BOUNDARY, inside the digital block.
//   The pixel is untouched - it asserts px_req and holds it until
//   acknowledged, exactly as in the baseline readouts.
//
//   Consequence, stated explicitly: without in-pixel memory the pixel is
//   released at readout rather than at the snapshot, so pixel dead time
//   and source-side loss are unchanged from the row/column baseline. The
//   timestamp benefit is obtained; the dead-time benefit is not.
//
//----------------------------------------------------------------------
// LOAD DETECTOR
//
//   Number of ACTIVE ROWS, i.e. an NROW-bit population count, not an
//   NPX-bit one. For a 64x64 array that is 64 bits instead of 4096, and
//   it is already the natural signal: the row lines exist anyway.
//   Congestion in a row/column readout is precisely many rows competing.
//
//   TH_ROWS > NROW disables the mechanism entirely, which turns this
//   module back into plain B1/B2. That is how the same bench measures
//   the baseline and the proposal under identical stimulus.
//======================================================================
module aer_hybrid #(
  parameter integer ROW_D   = 3,
  parameter integer COL_D   = 3,
  parameter integer TH_ROWS = 4,               // active rows that trigger a snapshot
  parameter real    T_SETTLE = 1.0,            // arbiter settling window [ns], see below
  parameter integer REQ_DLY = 4,
  parameter real    C_TPD   = 0.020,
  parameter real    M_TPD   = 0.020,
  parameter integer NROW    = (1 << ROW_D),
  parameter integer NCOL    = (1 << COL_D),
  parameter integer NPX     = NROW * NCOL,
  parameter integer AW      = ROW_D + COL_D
)(
  input  wire            rst_n,
  input  wire [NPX-1:0]  px_req,
  output wire [NPX-1:0]  px_ack,
  output wire            out_req,
  output wire [AW-1:0]   out_addr,
  input  wire            out_ack,

  // snapshot marker. Every event read out while hold_mode is high is
  // timestamped at the most recent hold_pulse, not at its own readout.
  output reg             hold_pulse,
  output reg             hold_mode
);

  //--------------------------------------------------------------------
  // snapshot register + effective request seen by the arbiters
  //--------------------------------------------------------------------
  reg  [NPX-1:0] held;
  reg            snap;
  integer        hi;

  //--------------------------------------------------------------------
  // Snapshot bits are retired on DOWNSTREAM ACCEPTANCE, not on px_ack.
  //
  //   px_ack = row_g & col_g & cin, so clearing held from px_ack closes
  //   a combinational loop:
  //       cin -> px_ack -> held -> eff_req -> grants -> granted -> cin
  //   The grant vanishes in the same instant cin asserts. A C-element
  //   latches on its input and then holds, so its output still rises one
  //   propagation delay later, by which time no pixel is selected,
  //   px_ack is zero everywhere, and the address flop captures a value
  //   with no source behind it.
  //
  //   The loop cannot be broken by simply holding held while cin is
  //   high: granted would stay asserted, cin could never fall and the
  //   four-phase would deadlock. The retire event must therefore be a
  //   genuinely later one. out_ack is that event - the parent has taken
  //   the address, addr_q names the pixel that was served, and the grant
  //   stays valid for the whole interval in between.
  //
  //   The pixel itself is still released early, at px_ack, so the
  //   semi-decoupled behaviour of the readout is unchanged.
  //--------------------------------------------------------------------
  always @(*) begin
    for (hi = 0; hi < NPX; hi = hi + 1) begin
      if (!rst_n)                             held[hi] = 1'b0;
      else if (snap)                          held[hi] = px_req[hi] & ~px_ack[hi];
      else if (out_ack && (hi == addr_q))     held[hi] = 1'b0;
      // else hold
    end
  end

  wire [NPX-1:0] eff_req = hold_mode ? held : px_req;

  //--------------------------------------------------------------------
  // row activity + load detector
  //--------------------------------------------------------------------
  wire [NROW-1:0] row_req;
  genvar r, c;
  generate
    for (r = 0; r < NROW; r = r + 1) begin : g_rowor
      assign row_req[r] = |eff_req[r*NCOL +: NCOL];
    end
  endgenerate

  // load is measured on the RAW requests: while draining a snapshot the
  // effective set only shrinks, so it must not be what decides whether
  // the next snapshot is needed.
  integer rn, act_rows;
  always @(*) begin
    act_rows = 0;
    for (rn = 0; rn < NROW; rn = rn + 1)
      if (|px_req[rn*NCOL +: NCOL]) act_rows = act_rows + 1;
  end
  wire trig = (act_rows >= TH_ROWS);

  //--------------------------------------------------------------------
  // snapshot control
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  // SETTLING GATE - required for correct operation.
  //
  //   B1/B2 change their request set ONE PIXEL AT A TIME, so the arbiter
  //   trees only re-settle locally. Switching eff_req between px_req and
  //   held changes the whole set AT ONCE: both trees re-arbitrate from
  //   scratch, grants glitch on the way, and |col_g glitching produces a
  //   SPURIOUS cin edge that latches a garbage address.
  //
  //   tracked hold% exactly - none at hold 0 %, first errors at hold
  //   1.1 %, heavy at hold 35 %+.
  //
  //   So the output stage is frozen while the trees settle. T_SETTLE
  //   must exceed the serial path row-tree -> column lines -> col-tree,
  //   i.e. about (ROW_D+COL_D) x 4 x M_TPD ~ 0.5 ns at the defaults.
  //   It is a parameter so the margin can be swept the same way C_TPD is.
  //
  //   Cost: 2 x T_SETTLE per snapshot cycle, amortised over the whole
  //   drained set (tens of events at high load).
  //--------------------------------------------------------------------
  reg settle_n;

  initial begin
    snap = 1'b0; hold_mode = 1'b0; hold_pulse = 1'b0; settle_n = 1'b1;
  end

  //--------------------------------------------------------------------
  // Reconfiguring the arbiter input set is only safe while the output
  // stage is genuinely quiescent, and `cin == 0` does not mean that.
  //
  //   A C-element latches on its inputs and holds. Once granted and
  //   cin_en coincide, even briefly, the internal state is set and the
  //   output will assert one propagation delay later no matter what
  //   happens to granted in the meantime. Sampling cin therefore reports
  //   "idle" during that delay while a transaction is already committed.
  //   Switching eff_req in that window moves the grants out from under
  //   the committed transaction: cin asserts with no pixel selected,
  //   px_ack is zero everywhere and the address flop captures a value
  //   with no source.
  //
  //   The sequence below closes that window:
  //     1. drop settle_n first, so no NEW transaction can be committed
  //     2. wait out the C-element propagation delay, letting any already
  //        committed transaction resolve into cin while the old grants
  //        are still valid
  //     3. wait for cin and cout to return to zero, i.e. that
  //        transaction has been latched, acknowledged and handed off
  //     4. only then change eff_req
  //--------------------------------------------------------------------
  initial begin : ctrl
    @(posedge rst_n);
    forever begin
      // ---- enter hold ------------------------------------------------
      wait (trig === 1'b1);
      settle_n = 1'b0;
      #(3.0*C_TPD);
      wait (cin  === 1'b0);
      wait (cout === 1'b0);

      snap       = 1'b1;
      hold_mode  = 1'b1;
      hold_pulse = 1'b1;
      #0.010;
      snap       = 1'b0;
      hold_pulse = 1'b0;
      #(T_SETTLE);
      settle_n   = 1'b1;

      // ---- leave hold once the snapshot is drained -------------------
      wait (held == {NPX{1'b0}});
      settle_n = 1'b0;
      #(3.0*C_TPD);
      wait (cin  === 1'b0);
      wait (cout === 1'b0);

      hold_mode = 1'b0;
      #(T_SETTLE);
      settle_n  = 1'b1;
    end
  end

  //--------------------------------------------------------------------
  // readout datapath - identical to aer_rowcol
  //--------------------------------------------------------------------
  wire [NROW-1:0]  row_g;
  wire [ROW_D-1:0] row_addr;
  wire             row_any;

  arb_tree #(.DEPTH(ROW_D), .M_TPD(M_TPD)) u_rowarb (
    .req(row_req), .gin(1'b1), .rup(row_any), .grant(row_g), .addr(row_addr));

  reg [NCOL-1:0] col_req;
  integer rr, cc;
  always @(*) begin
    for (cc = 0; cc < NCOL; cc = cc + 1) begin
      col_req[cc] = 1'b0;
      for (rr = 0; rr < NROW; rr = rr + 1)
        col_req[cc] = col_req[cc] | (row_g[rr] & eff_req[rr*NCOL + cc]);
    end
  end

  wire [NCOL-1:0]  col_g;
  wire [COL_D-1:0] col_addr;
  wire             col_any;

  arb_tree #(.DEPTH(COL_D), .M_TPD(M_TPD)) u_colarb (
    .req(col_req), .gin(1'b1), .rup(col_any), .grant(col_g), .addr(col_addr));

  //--------------------------------------------------------------------
  // An event exists only while a row AND a column are both granted.
  //
  //   col_req is derived from row_g, so the column tree responds one
  //   propagation delay after the row tree. During that lag row_g can
  //   already be zero while col_g is still asserted. Qualifying `granted`
  //   on the column grant alone would let cin rise inside that window,
  //   latching an address while px_ack = row_g & col_g & cin is zero for
  //   every pixel - an emission with no acknowledged source.
  //
  //   Both vectors are one-hot, so (|row_g) & (|col_g) is exactly "some
  //   pixel is selected".
  //--------------------------------------------------------------------
  wire granted = (|row_g) & (|col_g) & settle_n;
  wire cin, cout;
  wire cout_n  = ~cout;
  wire r_ack_n = ~out_ack;
  wire cin_en  = cout_n & r_ack_n;

  c_element #(.TPD_NS(C_TPD)) u_cin  (.rst_n(rst_n), .a(granted), .b(cin_en),  .z(cin));
  c_element #(.TPD_NS(C_TPD)) u_cout (.rst_n(rst_n), .a(cin),     .b(r_ack_n), .z(cout));

  generate
    for (r = 0; r < NROW; r = r + 1) begin : g_ackr
      for (c = 0; c < NCOL; c = c + 1) begin : g_ackc
        assign px_ack[r*NCOL + c] = row_g[r] & col_g[c] & cin;
      end
    end
  endgenerate

  reg [AW-1:0] addr_q;
  always @(posedge cin or negedge rst_n) begin
    if (!rst_n) addr_q <= {AW{1'b0}};
    else        addr_q <= {row_addr, col_addr};
  end
  assign out_addr = addr_q;

  delay_chain #(.N(REQ_DLY)) u_req_dly (.in(cout), .out(out_req));

endmodule
