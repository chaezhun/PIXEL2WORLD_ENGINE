`timescale 1ns/1ps
//======================================================================
// c_element - Muller C-element
//
//     z = 1  when a = 1 and b = 1
//     z = 0  when a = 0 and b = 0
//     z holds otherwise
//
//   Two implementations, selected by `AER_STRUCT_C:
//     behavioural  - inferred latch, deterministic in simulation
//     structural   - Earle-latch form, 3 AND + 1 OR
//                    [J. P. Murphy, "Design of latch-based C-element,"
//                     Electronics Letters 48(19), 2012]
//
//   RESET PLACEMENT (structural)
//     The feedback is taken from z, i.e. from after the reset gate:
//         z = rst_n & ( (a & b) | (a & z) | (b & z) )
//     Feeding back from before the reset gate leaves the state node
//     uninitialised - with a = 0 the loop reduces to z = b & z and
//     holds X from time zero, which propagates through the request
//     path the moment reset is released.
//
//   OUTPUT DELAY
//     TPD_NS must be non-zero. cin -> cout -> ~cout -> cin forms a
//     feedback loop between two C-elements; with zero delay the loop
//     resolves inside one simulation time step and the address flop can
//     be clocked by an unresolved value. A real standard cell has
//     propagation delay, so a non-zero value is the faithful model. It
//     also makes the C-element response time an explicit, sweepable
//     quantity for the handover-window margin.
//======================================================================
module c_element #(
  // Output propagation delay [ns] for the behavioural model.
  //   Must be non-zero; see the OUTPUT DELAY note in the header.
  parameter real TPD_NS = 0.020
)(
  input  wire rst_n,   // async low reset -> forces z = 0
  input  wire a,
  input  wire b,
  output wire z
);

`ifdef AER_STRUCT_C
  //--------------------------------------------------------------------
  // Earle-latch C-element:  z = rst_n & ( (a&b) | (a&z) | (b&z) )
  // Feedback is taken from z so that reset initialises the state node.
  //--------------------------------------------------------------------
  (* dont_touch = "true" *) wire n_ab, n_az, n_bz, z_pre;

  and u_ab (n_ab, a, b);
  and u_az (n_az, a, z);
  and u_bz (n_bz, b, z);
  or  u_or (z_pre, n_ab, n_az, n_bz);
  and #(TPD_NS) u_rst (z, z_pre, rst_n);

`else
  //--------------------------------------------------------------------
  // behavioural : intentional latch, deterministic in simulation
  //--------------------------------------------------------------------
  reg z_r;

  always @(*) begin
    if (!rst_n)        z_r = 1'b0;
    else if ( a &&  b) z_r = 1'b1;
    else if (!a && !b) z_r = 1'b0;
    // else: hold  (this is the C-element's memory)
  end

  // non-zero delay: breaks the zero-delay delta-cycle loop (see TPD_NS)
  assign #(TPD_NS) z = z_r;
`endif

endmodule
