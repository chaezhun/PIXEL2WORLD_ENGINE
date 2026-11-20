`timescale 1ns/1ps
//======================================================================
// aer_node - 2-input arbitered address-event encoder node
//
//   One node of the binary encoder tree. Each node arbitrates between
//   two child channels, appends its own select bit as the MSB of the
//   address, and forwards the result to its parent.
//
//   Protocol : 4-phase bundled data
//   Control  : semi-decoupled dual C-element
//              [S. Furber and P. Day, "Four-phase micropipeline latch
//               control circuits," IEEE TVLSI 4(2):247-253, 1996]
//   Arbiter  : cross-coupled NAND mutex per node
//   Storage  : edge-triggered flip-flop clocked by the LOCALLY GENERATED
//              acknowledge, not by the incoming request. This is what
//              allows a standard-cell flow, and it is also why static
//              timing analysis needs create_generated_clock and
//              set_max_delay rather than an ordinary clock definition.
//
//----------------------------------------------------------------------
// CONTROL EQUATIONS
//
//     cin_en = ~cout & ~r_ack        output channel fully idle
//     cin    = C(granted, cin_en)    clocks the address FF, acks a child
//     cout   = C(cin,     ~r_ack)    drives the request to the parent
//
//   Both C-elements are required, and cin_en must include ~r_ack.
//   Two failure modes justify this:
//
//   (a) Single C-element, cin = C(granted, ~r_ack).
//       The node fires, the child sees its acknowledge and releases, so
//       granted falls and the child is free to send a new event. r_ack
//       has not arrived, so cin is still high; the new request raises
//       granted again and C(1,1) leaves cin high. No rising edge is
//       produced, the flip-flop never captures the new address, yet
//       l_ack = g & cin is asserted immediately and the child believes
//       the transfer succeeded. The event is lost silently.
//
//   (b) Dual C-element with cin_en = ~cout only.
//       cout falls as soon as (cin = 0 and r_ack = 1), but r_ack is not
//       withdrawn until r_req has propagated through the matched delay
//       and the parent has responded. Across that interval cout = 0
//       while r_ack = 1, so a new event is admitted into a channel whose
//       previous four-phase has not returned to zero, and case (a)
//       repeats one stage later. The defect appears only at the node
//       with the longest acknowledge turnaround, so small or lightly
//       loaded test cases do not expose it.
//
//   Including ~r_ack costs a little input/output overlap and preserves
//   the semi-decoupled property that matters: the child's own four-phase
//   completes without waiting for r_ack.
//
//----------------------------------------------------------------------
// BUNDLED-DATA CONSTRAINT
//
//   Over all PVT corners:
//     t(cout -> r_req) >= t(cin -> data_q) + t(parent MUX)
//                         + t_setup(parent FF) - t(cin -> cout)
//   REQ_DLY is the parameter used to satisfy this inequality.
//
//   GNT_GAP is retained for use with a real delay element only. The
//   behavioural delay_chain is a lumped inertial delay, so widening it
//   swallows the handover pulse instead of extending it.
//======================================================================
module aer_node #(
  parameter integer IN_W    = 0,                        // child address width (0 at leaf level)
  parameter integer REQ_DLY = 4,                        // matched delay stages on r_req
  parameter integer GNT_GAP = 0,                        // >0 : see WARNING above
  parameter real    C_TPD   = 0.020,                    // C-element output delay [ns], must be > 0
  parameter real    M_TPD   = 0.020,                    // mutex base gate delay [ns]; handover window scales with it
  parameter integer DW      = (IN_W > 0) ? IN_W : 1,    // padded child bus width
  parameter integer OUT_W   = IN_W + 1                  // this node's address width
)(
  input  wire              rst_n,

  // ---- child channel 0 (address bit = 0) ----
  input  wire              l0_req,
  input  wire [DW-1:0]     l0_data,
  output wire              l0_ack,

  // ---- child channel 1 (address bit = 1) ----
  input  wire              l1_req,
  input  wire [DW-1:0]     l1_data,
  output wire              l1_ack,

  // ---- parent channel ----
  output wire              r_req,
  output wire [OUT_W-1:0]  r_data,
  input  wire              r_ack
);

  //--------------------------------------------------------------------
  // 1. arbitration
  //--------------------------------------------------------------------
  wire g0, g1;
  mutex #(.TPD_NS(M_TPD)) u_mutex (
    .r1 (l0_req),
    .r2 (l1_req),
    .g1 (g0),
    .g2 (g1)
  );

  wire sel = g1;                 // 0 -> child 0, 1 -> child 1

  wire granted_raw = g0 | g1;
  wire granted;
  generate
    if (GNT_GAP == 0) begin : g_nogap
      assign granted = granted_raw;
    end else begin : g_gap
      // NOTE: only meaningful with a real delay element. See the
      // GNT_GAP warning in the header - the behavioural delay_chain is
      // inertial and SWALLOWS the handover pulse instead of widening it.
      delay_chain #(.N(GNT_GAP)) u_gap (.in(granted_raw), .out(granted));
    end
  endgenerate

  //--------------------------------------------------------------------
  // 2. semi-decoupled dual C-element control
  //--------------------------------------------------------------------
  wire cin, cout;
  wire cout_n  = ~cout;
  wire r_ack_n = ~r_ack;

  //--------------------------------------------------------------------
  // INPUT ENABLE - the output channel must be FULLY idle, i.e. the
  // request retracted AND the acknowledge withdrawn.
  //
  //   ~cout alone is NOT enough, and that bug is measurable. cout falls
  //   as soon as (cin=0 & r_ack=1), but r_ack does not withdraw until
  //   r_req has propagated down (REQ_DLY) and the parent has responded.
  //   Across that gap cout=0 while r_ack=1, so C(granted, ~cout) admits
  //   a new event into a channel whose previous four-phase has not
  //   returned to zero. cin then cannot fall again - C(0, ~cout=1) holds
  //   at 1 - so the following child sees l_ack = g & cin = 1 straight
  //   away, believes it was accepted, and vanishes. Silent loss.
  //
  //:
  //     with b = cout_n        PHASE 2 root latched 5 of 8;
  //                            PHASE 3 lvl0 10003, lvl1 10003, root 9975
  //                            -> every lost event died at the root, the
  //                               node with the longest ack turnaround.
  //
  //   Gating on r_ack_n as well costs a little input/output overlap but
  //   keeps the semi-decoupled property that matters: the child's own
  //   four-phase still completes without waiting for r_ack (see the
  //   cin-falls step below).
  //--------------------------------------------------------------------
  wire cin_en = cout_n & r_ack_n;

  // input side : acknowledges the children, clocks the address FF
  c_element #(.TPD_NS(C_TPD)) u_cin (
    .rst_n (rst_n),
    .a     (granted),
    .b     (cin_en),
    .z     (cin)
  );

  // output side : drives the request to the parent
  c_element #(.TPD_NS(C_TPD)) u_cout (
    .rst_n (rst_n),
    .a     (cin),
    .b     (r_ack_n),
    .z     (cout)
  );

  assign l0_ack = g0 & cin;
  assign l1_ack = g1 & cin;

  //--------------------------------------------------------------------
  // 3. datapath : concatenate this node's select bit as the MSB
  //--------------------------------------------------------------------
  wire [OUT_W-1:0] data_next;

  generate
    if (IN_W == 0) begin : g_leaf
      assign data_next = sel;                              // 1 bit total
    end else begin : g_inner
      wire [DW-1:0] mux_out = sel ? l1_data : l0_data;
      assign data_next = {sel, mux_out};                   // IN_W+1 bits
    end
  endgenerate

  // FF clocked by the locally generated acknowledge.
  // Deliberately a generated clock: this is why STA needs
  // create_generated_clock / set_max_delay instead of a normal clock.
  reg [OUT_W-1:0] data_q;
  always @(posedge cin or negedge rst_n) begin
    if (!rst_n) data_q <= {OUT_W{1'b0}};
    else        data_q <= data_next;
  end

  assign r_data = data_q;

  //--------------------------------------------------------------------
  // 4. matched delay on the outgoing request (bundled data)
  //
  //    Must satisfy, over ALL PVT corners:
  //      t(cout -> r_req)  >=  t(cin -> data_q) + t(mux at parent)
  //                            + t_setup(parent FF)  -  t(cin -> cout)
  //    This inequality is the robustness (강건설계) claim of the design
  //    and is what scripts/timing_closure.sh iterates REQ_DLY to meet.
  //--------------------------------------------------------------------
  delay_chain #(.N(REQ_DLY)) u_req_dly (
    .in  (cout),
    .out (r_req)
  );

endmodule
