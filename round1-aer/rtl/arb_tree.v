`timescale 1ns/1ps
//======================================================================
// arb_tree - level-mode arbiter tree with one-hot grant
//
//   The traditional AER arbiter of the Sivilotti / Mahowald / Lazzaro
//   lineage, formalised in K. A. Boahen, "Point-to-Point Connectivity
//   Between Neuromorphic Chips Using Address Events," IEEE TCAS-II
//   47(5):416-434, 2000.
//
//   This is a different primitive from the micropipeline encoder in
//   aer_node.v:
//     aer_node  - 4-phase micropipeline; each stage LATCHES an address
//                 and hands it on, so events are pipelined
//     arb_tree  - purely combinational grant distribution over mutex
//                 state; requests flow up as an OR, the grant flows down
//                 and is HELD while the winning request is held
//
//   Per node, following Boahen's recursive decomposition
//     ARB(X) = ARB(floor(X/2)) || ARB(2) || ARB(X - floor(X/2)):
//
//     rup  = r0 | r1                    request to the parent
//     g0   = gin & mutex_g0             grant down, gated by the parent
//     g1   = gin & mutex_g1
//     addr = {sel, sel ? a1 : a0}       sel is this node's address bit
//
//   N inputs require N-1 two-input mutex cells and log2(N) levels.
//   grant[] is one-hot while gin is high.
//======================================================================
module arb_tree #(
  parameter integer DEPTH = 3,
  parameter real    M_TPD = 0.020,
  parameter integer N     = (1 << DEPTH)   // derived - do not override
)(
  input  wire [N-1:0]     req,
  input  wire             gin,     // grant enable from the parent
  output wire             rup,     // request to the parent
  output wire [N-1:0]     grant,   // one-hot while gin is high
  output wire [DEPTH-1:0] addr     // index of the granted input
);

  generate
    if (DEPTH == 1) begin : g_leaf
      wire m0, m1;
      mutex #(.TPD_NS(M_TPD)) u_m (
        .r1 (req[0]), .r2 (req[1]),
        .g1 (m0),     .g2 (m1)
      );
      assign rup      = req[0] | req[1];
      assign grant[0] = gin & m0;
      assign grant[1] = gin & m1;
      assign addr     = m1;

    end else begin : g_node
      localparam integer H = N/2;

      wire             r0, r1, m0, m1;
      wire [DEPTH-2:0] a0, a1;

      arb_tree #(.DEPTH(DEPTH-1), .M_TPD(M_TPD)) u0 (
        .req   (req[H-1:0]),
        .gin   (gin & m0),
        .rup   (r0),
        .grant (grant[H-1:0]),
        .addr  (a0)
      );

      arb_tree #(.DEPTH(DEPTH-1), .M_TPD(M_TPD)) u1 (
        .req   (req[N-1:H]),
        .gin   (gin & m1),
        .rup   (r1),
        .grant (grant[N-1:H]),
        .addr  (a1)
      );

      mutex #(.TPD_NS(M_TPD)) u_m (
        .r1 (r0), .r2 (r1),
        .g1 (m0), .g2 (m1)
      );

      assign rup  = r0 | r1;
      assign addr = {m1, m1 ? a1 : a0};
    end
  endgenerate

endmodule
