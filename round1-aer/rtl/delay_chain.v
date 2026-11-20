`timescale 1ns/1ps
//======================================================================
// delay_chain - matched delay element for the bundled-data request line
//
//   Bundled data requires  t(req path) >= t(data path) + t_setup.
//   N is the tuning knob used to satisfy that inequality.
//
//   Two implementations, selected by `AER_STRUCT_DLY:
//     behavioural  - lumped inertial delay
//     structural   - buffer chain with dont_touch, per-stage delay
//
//   The structural form must carry a per-stage delay. With zero-delay
//   buffers the chain is transparent, REQ_DLY has no effect and the
//   bundled-data margin goes untested.
//
//   ASIC NOTE
//     Replace with the PDK's characterised delay cells. Delay built
//     from ordinary logic buffers tracks PVT poorly, which is the
//     robustness axis this design is evaluated on.
//
//   FPGA NOTE
//     A buffer chain is not a reliable delay on an FPGA; the tool
//     absorbs it into LUTs and routing. For functional simulation the
//     behavioural form is the correct default.
//======================================================================
module delay_chain #(
  parameter integer N      = 4,      // number of delay stages
  parameter real    TPD_NS = 0.10    // per-stage delay (behavioural sim)
)(
  input  wire in,
  output wire out
);

`ifdef AER_STRUCT_DLY
  //--------------------------------------------------------------------
  // structural buffer chain
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  // Primitive delays are simulation-only (synthesis ignores them). They
  // must be here anyway: with zero-delay bufs the chain is transparent,
  // so REQ_DLY has NO effect in simulation and the bundled-data margin
  // of the bundled-data constraint is silently untested - the structural build
  // would "pass" for the wrong reason.
  //--------------------------------------------------------------------
  (* dont_touch = "true" *) wire [N:0] c;
  assign c[0] = in;

  genvar i;
  generate
    for (i = 0; i < N; i = i + 1) begin : g_buf
      (* dont_touch = "true" *) buf #(TPD_NS) u_b (c[i+1], c[i]);
    end
  endgenerate

  assign out = c[N];

`else
  //--------------------------------------------------------------------
  // behavioural lumped delay
  //--------------------------------------------------------------------
  assign #(N*TPD_NS) out = in;
`endif

endmodule
