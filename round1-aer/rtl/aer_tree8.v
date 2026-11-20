`timescale 1ns/1ps
//======================================================================
// aer_tree8 : 8-event, 3-stage binary-tree AER encoder
//
//   Same size as the R26 prototype (arXiv:2604.05313: "an 8-event AER
//   template was implemented, fabricated, and characterized"), so the
//   T1 numbers can be put straight next to their silicon results:
//     33 MEvent/s, 50 ns event latency, 16.67 ns/(event-bit),
//     435 fJ/event, 145 fJ/(event-bit), 8.7 uW, 65.57 x 44.6 um^2 @65nm
//
//   Address assembly (LSB generated first):
//     level 0 node picks within a pair        -> addr[0]
//     level 1 node picks the pair             -> addr[1]
//     level 2 node picks the half             -> addr[2]
//     out_addr = {sel2, sel1, sel0} = event index
//
//   T1 SCOPE: this is a flow-risk probe, not the contest deliverable.
//   The question it answers is only:
//     "can Genus/Innovus close timing on an async handshake loop with
//      FFs clocked by a locally generated ack, using the R26 iterative
//      set_max_delay procedure?"
//======================================================================
module aer_tree8 #(
  parameter integer REQ_DLY = 4,
  parameter real    C_TPD   = 0.020,  // C-element output delay [ns] - see c_element.v
  parameter real    M_TPD   = 0.020   // mutex base gate delay [ns]  - see mutex.v
)(
  input  wire        rst_n,

  // 8 event sources (4-phase req/ack, no data)
  input  wire [7:0]  ev_req,
  output wire [7:0]  ev_ack,

  // encoded output channel
  output wire        out_req,
  output wire [2:0]  out_addr,
  input  wire        out_ack
);

  //--------------------------------------------------------------------
  // level 0 : 4 nodes, IN_W=0 -> 1-bit address out
  //--------------------------------------------------------------------
  wire [3:0]  l0_req_o;
  wire [3:0]  l0_ack_i;
  wire [0:0]  l0_data_o [3:0];

  genvar i;
  generate
    for (i = 0; i < 4; i = i + 1) begin : g_lvl0
      aer_node #(.IN_W(0), .REQ_DLY(REQ_DLY), .C_TPD(C_TPD), .M_TPD(M_TPD)) u_n (
        .rst_n   (rst_n),
        .l0_req  (ev_req[2*i]),
        .l0_data (1'b0),
        .l0_ack  (ev_ack[2*i]),
        .l1_req  (ev_req[2*i+1]),
        .l1_data (1'b0),
        .l1_ack  (ev_ack[2*i+1]),
        .r_req   (l0_req_o[i]),
        .r_data  (l0_data_o[i]),
        .r_ack   (l0_ack_i[i])
      );
    end
  endgenerate

  //--------------------------------------------------------------------
  // level 1 : 2 nodes, IN_W=1 -> 2-bit address out
  //--------------------------------------------------------------------
  wire [1:0]  l1_req_o;
  wire [1:0]  l1_ack_i;
  wire [1:0]  l1_data_o [1:0];

  generate
    for (i = 0; i < 2; i = i + 1) begin : g_lvl1
      aer_node #(.IN_W(1), .REQ_DLY(REQ_DLY), .C_TPD(C_TPD), .M_TPD(M_TPD)) u_n (
        .rst_n   (rst_n),
        .l0_req  (l0_req_o[2*i]),
        .l0_data (l0_data_o[2*i]),
        .l0_ack  (l0_ack_i[2*i]),
        .l1_req  (l0_req_o[2*i+1]),
        .l1_data (l0_data_o[2*i+1]),
        .l1_ack  (l0_ack_i[2*i+1]),
        .r_req   (l1_req_o[i]),
        .r_data  (l1_data_o[i]),
        .r_ack   (l1_ack_i[i])
      );
    end
  endgenerate

  //--------------------------------------------------------------------
  // level 2 : 1 node, IN_W=2 -> 3-bit address out
  //--------------------------------------------------------------------
  aer_node #(.IN_W(2), .REQ_DLY(REQ_DLY), .C_TPD(C_TPD), .M_TPD(M_TPD)) u_root (
    .rst_n   (rst_n),
    .l0_req  (l1_req_o[0]),
    .l0_data (l1_data_o[0]),
    .l0_ack  (l1_ack_i[0]),
    .l1_req  (l1_req_o[1]),
    .l1_data (l1_data_o[1]),
    .l1_ack  (l1_ack_i[1]),
    .r_req   (out_req),
    .r_data  (out_addr),
    .r_ack   (out_ack)
  );

endmodule
