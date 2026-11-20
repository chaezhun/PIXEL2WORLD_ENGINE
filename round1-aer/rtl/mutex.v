`timescale 1ns/1ps
//======================================================================
// mutex - 2-input mutual exclusion element (arbiter cell)
//
//   Reference : C. L. Seitz, "System Timing," in Mead & Conway,
//               Introduction to VLSI Systems, Addison-Wesley, 1980,
//               ch. 7, pp. 218-262.
//
//   Guarantees
//     - at most one grant asserted at any time
//     - a grant is held until its request is released
//     - a grant rises only while the other grant is low
//
//   Two implementations, selected by `AER_STRUCT_MUTEX:
//     behavioural  - deterministic, simulation-safe, LFSR tie-break
//     structural   - cross-coupled NAND pair, the synthesis target
//
//   HANDOVER WINDOW
//     When a grant passes to the waiting sibling, both grants must be
//     low for a short interval. The downstream C-element uses that
//     interval to return to zero; without it the next event receives no
//     rising edge and is lost silently. Window width:
//         structural   2.0 x TPD_NS
//         behavioural  2.5 x TPD_NS   (asymmetric output delay)
//     The requirement is  window > C-element response time.
//
//   METASTABILITY
//     In silicon a simultaneous tie is resolved by device mismatch and
//     thermal noise inside the metastable window, so the outcome is
//     stochastic. No RTL model reproduces this. The behavioural model
//     uses an LFSR so that fairness statistics are not dominated by a
//     fixed bias; the structural model uses MISMATCH_NS, a small skew
//     between the two NAND gates, which is what static device mismatch
//     does in a real cell. A standard-cell flow cannot guarantee
//     arbiter behaviour: silicon requires a characterised arbiter cell
//     with an analogue metastability filter.
//======================================================================
module mutex #(
  // Base gate delay [ns] for the simulation models.
  //   Handover window : 2.0 x TPD_NS (structural) / 2.5 x TPD_NS (behav.)
  //   Requirement     : window > C-element response (c_element.TPD_NS)
  //   Default 20 ps gives a 2x-2.5x margin over the 20 ps C-element.
  parameter real TPD_NS = 0.020,

  // Device-mismatch skew [ns] between the two cross-coupled NAND gates.
  //   Must be non-zero. With identical delays the pair is exactly
  //   symmetric on a simultaneous tie: n1 and n2 fall together, rise
  //   together and oscillate with period 4 x TPD_NS instead of latching.
  //   While n1 == n2 both grants read 0, every node presents sel = 0 and
  //   the encoded address degenerates to all-zeros.
  //   Static device mismatch performs this role in a real cell. 1 ps is
  //   far below the handover window, so the margin is unaffected.
  parameter real MISMATCH_NS = 0.001
)(
  input  wire r1,
  input  wire r2,
  output wire g1,
  output wire g2
);

`ifdef AER_STRUCT_MUTEX
  //--------------------------------------------------------------------
  // structural : cross-coupled NAND pair + metastability filter
  //   primitive delays are simulation-only (synthesis ignores them) and
  //   prevent zero-delay oscillation on an exact tie
  //--------------------------------------------------------------------
  (* dont_touch = "true" *) wire n1, n2, n1_b, n2_b;

  // asymmetric on purpose - see MISMATCH_NS. u_n1 is the faster gate,
  // so on an exact tie n1 falls first, which forces n2 back to 1 before
  // its own pending transition matures (inertial delay cancels it) and
  // the pair latches at n1=0, n2=1 => g1 wins.
  nand #(2.0*TPD_NS)               u_n1 (n1, r1, n2);
  nand #(2.0*TPD_NS + MISMATCH_NS) u_n2 (n2, r2, n1);

  not  #(TPD_NS) u_i1 (n1_b, n1);
  not  #(TPD_NS) u_i2 (n2_b, n2);
  and  #(TPD_NS) u_g1 (g1, n1_b, n2);
  and  #(TPD_NS) u_g2 (g2, n2_b, n1);

`else
  //--------------------------------------------------------------------
  // behavioural : deterministic, simulation-safe
  //--------------------------------------------------------------------
  reg g1_r, g2_r;

  // pseudo-random tie-break source (approximates stochastic resolution)
  reg [7:0] lfsr = 8'hA5;
  wire      tie_pick = lfsr[0];

  always @(*) begin
    if (!r1 && !r2) begin
      g1_r = 1'b0;
      g2_r = 1'b0;
    end else if (r1 && !r2) begin
      // holding a grant for r2 is impossible here, safe to grant r1
      g1_r = 1'b1;
      g2_r = 1'b0;
    end else if (!r1 && r2) begin
      g1_r = 1'b0;
      g2_r = 1'b1;
    end else begin
      // both requesting: keep an existing grant, otherwise pick one
      if (!g1_r && !g2_r) begin
        g1_r = ~tie_pick;
        g2_r =  tie_pick;
      end
      // else hold current grant until its request drops
    end
  end

  // advance the LFSR once per granted transaction
  always @(posedge g1_r or posedge g2_r)
    lfsr <= {lfsr[6:0], lfsr[7] ^ lfsr[5] ^ lfsr[4] ^ lfsr[3]};

  //--------------------------------------------------------------------
  // Asymmetric output delay: slow assert, fast deassert.
  //   On a handover the falling grant clears before the rising one
  //   arrives, creating the both-grants-low window the downstream
  //   C-element needs. The structural NAND pair produces this window
  //   naturally; the behavioural model states it explicitly.
  //   Simulation proves function, not margin - the margin over PVT is a
  //   static-timing and SPICE question.
  //--------------------------------------------------------------------
  assign #(3.0*TPD_NS, 0.5*TPD_NS) g1 = g1_r;
  assign #(3.0*TPD_NS, 0.5*TPD_NS) g2 = g2_r;
`endif

endmodule
