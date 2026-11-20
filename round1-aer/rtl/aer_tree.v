`timescale 1ns/1ps
//======================================================================
// aer_tree - generic-depth binary-tree AER encoder
//
//   NCH = 2**DEPTH event inputs, DEPTH-bit output address.
//   out_addr equals the channel index: the root's select bit is the MSB
//   (0 selects the lower half) and each level below contributes one less
//   significant bit.
//
//   Written recursively. Address buses widen by one bit per level, which
//   a recursive instantiation expresses directly without oversized flat
//   vectors or variable-width part-selects.
//
//   Parameterising the depth allows the two quantities required by the
//   queueing model to be measured against array size rather than
//   assumed:
//     tau_0   fixed path delay,   grows with arbitration depth
//     Delta   service quantum,    set by the root/output cycle
//   Both feed the tile-partitioning analysis in the design report.
//======================================================================
module aer_tree #(
  parameter integer DEPTH   = 3,               // NCH = 2**DEPTH
  parameter integer REQ_DLY = 4,
  parameter real    C_TPD   = 0.020,
  parameter real    M_TPD   = 0.020,
  parameter integer NCH     = (1 << DEPTH)     // derived - do not override
)(
  input  wire              rst_n,

  input  wire [NCH-1:0]    ev_req,
  output wire [NCH-1:0]    ev_ack,

  output wire              r_req,
  output wire [DEPTH-1:0]  r_data,
  input  wire              r_ack
);

  generate
    if (DEPTH == 1) begin : g_leaf
      //----------------------------------------------------------------
      // bottom of the recursion: one node, two raw event inputs
      //----------------------------------------------------------------
      aer_node #(
        .IN_W    (0),
        .REQ_DLY (REQ_DLY),
        .C_TPD   (C_TPD),
        .M_TPD   (M_TPD)
      ) u_n (
        .rst_n   (rst_n),
        .l0_req  (ev_req[0]),
        .l0_data (1'b0),
        .l0_ack  (ev_ack[0]),
        .l1_req  (ev_req[1]),
        .l1_data (1'b0),
        .l1_ack  (ev_ack[1]),
        .r_req   (r_req),
        .r_data  (r_data),
        .r_ack   (r_ack)
      );

    end else begin : g_node
      //----------------------------------------------------------------
      // two half-size subtrees feeding one node.
      //   u_s0 takes the low half  -> this node's select bit is 0
      //   u_s1 takes the high half -> select bit is 1
      // so the assembled address is the channel index.
      //----------------------------------------------------------------
      localparam integer HALF = NCH/2;

      wire [DEPTH-2:0] d0, d1;
      wire             q0, q1, a0, a1;

      aer_tree #(
        .DEPTH   (DEPTH-1),
        .REQ_DLY (REQ_DLY),
        .C_TPD   (C_TPD),
        .M_TPD   (M_TPD)
      ) u_s0 (
        .rst_n   (rst_n),
        .ev_req  (ev_req[HALF-1:0]),
        .ev_ack  (ev_ack[HALF-1:0]),
        .r_req   (q0),
        .r_data  (d0),
        .r_ack   (a0)
      );

      aer_tree #(
        .DEPTH   (DEPTH-1),
        .REQ_DLY (REQ_DLY),
        .C_TPD   (C_TPD),
        .M_TPD   (M_TPD)
      ) u_s1 (
        .rst_n   (rst_n),
        .ev_req  (ev_req[NCH-1:HALF]),
        .ev_ack  (ev_ack[NCH-1:HALF]),
        .r_req   (q1),
        .r_data  (d1),
        .r_ack   (a1)
      );

      aer_node #(
        .IN_W    (DEPTH-1),
        .REQ_DLY (REQ_DLY),
        .C_TPD   (C_TPD),
        .M_TPD   (M_TPD)
      ) u_n (
        .rst_n   (rst_n),
        .l0_req  (q0),
        .l0_data (d0),
        .l0_ack  (a0),
        .l1_req  (q1),
        .l1_data (d1),
        .l1_ack  (a1),
        .r_req   (r_req),
        .r_data  (r_data),
        .r_ack   (r_ack)
      );
    end
  endgenerate

endmodule
