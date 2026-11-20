`timescale 1ns/1ps
//======================================================================
// aer_scan - synchronous scan readout (baseline B3)
//
//   Models the Samsung DVS Gen3 readout: the arbiter is replaced by a
//   scanner and a global hold captures the whole array at one instant.
//
//   1. ARBITER -> SCANNER. Order is fixed and known, so there is no
//      arbitration uncertainty, no mutex and no metastability.
//
//   2. GLOBAL HOLD. Every pixel is captured at one instant, so
//        - the pixel is released at the hold rather than at readout and
//          its dead time is at most one frame
//        - THE TIMESTAMP IS THE HOLD INSTANT, so every event in a frame
//          shares one timestamp and the timestamp error no longer
//          depends on readout order or on event rate
//
//   3. GLOBAL RESET. Clears the array after the frame so that events in
//      flight do not leak into the next one.
//
//   COST
//     Timestamp resolution is quantised to the frame period and the
//     error averages T_frame/2 even at zero event rate, where an arbiter
//     responds in tau_0. Static power is drawn regardless of activity,
//     and a full raster scan spends a clock on empty positions.
//
//   SCOPE
//     Full raster scan, one position per T_CLK. Gen3's row-parallel
//     column-vector readout removes the empty-position cost and is the
//     natural extension. T_CLK defaults to the measured asynchronous
//     service quantum so both families share the same peak output rate
//     and the comparison isolates structure from clock choice.
//======================================================================
module aer_scan #(
  parameter integer ROW_D   = 3,
  parameter integer COL_D   = 3,
  parameter real    T_CLK   = 1.86,          // ns per scanned position
  parameter integer REQ_DLY = 4,
  parameter integer NROW    = (1 << ROW_D),  // derived
  parameter integer NCOL    = (1 << COL_D),  // derived
  parameter integer NPX     = NROW * NCOL,   // derived
  parameter integer AW      = ROW_D + COL_D  // derived
)(
  input  wire            rst_n,

  input  wire [NPX-1:0]  px_req,
  output reg  [NPX-1:0]  px_ack,

  output reg             out_req,
  output reg  [AW-1:0]   out_addr,
  input  wire            out_ack,

  // frame marker : pulses at every global hold. The TIMESTAMP of every
  // event read out in the following frame is this instant, so a bench
  // must sample it here rather than at out_req.
  output reg             hold_pulse
);

  reg [NPX-1:0] hold;
  integer       pos;
  real          t_slot;

  initial begin
    px_ack     = {NPX{1'b0}};
    out_req    = 1'b0;
    out_addr   = {AW{1'b0}};
    hold_pulse = 1'b0;
    hold       = {NPX{1'b0}};
  end

  always @(*) if (!rst_n) begin
    px_ack     = {NPX{1'b0}};
    out_req    = 1'b0;
    hold_pulse = 1'b0;
  end

  initial begin : scanner
    @(posedge rst_n);
    forever begin
      //----------------------------------------------------------------
      // GLOBAL HOLD + GLOBAL RESET
      //   capture the whole array at one instant, then acknowledge every
      //   captured pixel at once so it can re-arm immediately. The pixel
      //   is therefore free during the entire scan, which is what caps
      //   its dead time at one frame.
      //----------------------------------------------------------------
      hold       = px_req;
      hold_pulse = 1'b1;
      px_ack     = hold;              // global reset pulse
      #0.010;
      hold_pulse = 1'b0;
      px_ack     = {NPX{1'b0}};

      //----------------------------------------------------------------
      // RASTER SCAN : every position costs T_CLK whether or not it holds
      // an event. If the output handshake overruns the slot the scanner
      // stalls, which is the interface-bandwidth limit of [S1] slide 12.
      //----------------------------------------------------------------
      for (pos = 0; pos < NPX; pos = pos + 1) begin
        t_slot = $realtime;
        if (hold[pos]) begin
          out_addr = pos[AW-1:0];
          #(REQ_DLY * 0.1);           // bundled data: address first
          out_req = 1'b1;
          wait (out_ack === 1'b1);
          out_req = 1'b0;
          wait (out_ack === 1'b0);
        end
        if (($realtime - t_slot) < T_CLK) #(T_CLK - ($realtime - t_slot));
      end
    end
  end

endmodule
