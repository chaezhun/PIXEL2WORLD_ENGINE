`timescale 1ns/1ps
//======================================================================
// aer_rowcol - row/column arbiter readout (baselines B1 and B2)
//
//   Traditional 2D readout: a row arbiter selects a row, a SHARED column
//   arbiter selects one pixel within it.
//
//   AREA
//     A flat tree over NROW*NCOL pixels needs NROW*NCOL-1 mutex cells.
//     Row/column needs (NROW-1)+(NCOL-1), i.e. Boahen's reduction from
//     N to sqrt(N). For 64x64, 4095 cells versus 126.
//
//   COST
//     Column lines are shared, so only one row may drive them at a time
//     and the column tree is a single non-pipelined resource.
//
//   B1 versus B2
//     A plain arbiter drains a row either way, because row_req[r] stays
//     high while that row has pending pixels and the same row keeps
//     winning. Word-parallel and word-serial burst mode therefore share
//     this readout circuit and differ only in address encoding:
//         word-parallel  ROW_D + COL_D bits every event
//         word-serial    ROW_D once per burst, COL_D per event
//                        -> COL_D + ROW_D/k bits per event
//     k, the events per row grant, is measured in the testbench.
//
//   READOUT SEQUENCE
//     1. pixel (r,c) asserts px_req and holds it (level, as a pixel
//        front-end behaves - it is not a four-phase source)
//     2. row_req[r] = OR of that row's pixels        -> row arb_tree
//     3. row_g[r] one-hot grant selects a row
//     4. col_req[c] = px_req of the granted row only -> column arb_tree
//     5. col_g[c] selects one pixel in that row
//     6. the output stage latches {row_addr, col_addr} and acknowledges
//        pixel (r,c), which releases px_req
//
//   OUTPUT STAGE
//     Identical to aer_node - same semi-decoupled dual C-element, matched
//     delay and address flop. Keeping it identical ensures a throughput
//     comparison against the flat tree measures the arbitration
//     structure rather than the output interface.
//
//     `granted` is taken from the column GRANT, not from the request OR.
//     The request OR stays high while any pixel is pending, so it would
//     never return to zero and cin could never fall. The grant passes
//     through the mutex handover window on every event, which is the
//     both-grants-low interval cin requires.
//======================================================================
module aer_rowcol #(
  parameter integer ROW_D   = 3,               // NROW = 2**ROW_D
  parameter integer COL_D   = 3,               // NCOL = 2**COL_D
  parameter integer REQ_DLY = 4,
  parameter real    C_TPD   = 0.020,
  parameter real    M_TPD   = 0.020,
  parameter integer NROW    = (1 << ROW_D),    // derived
  parameter integer NCOL    = (1 << COL_D),    // derived
  parameter integer NPX     = NROW * NCOL,     // derived
  parameter integer AW      = ROW_D + COL_D    // derived
)(
  input  wire            rst_n,

  // pixel array : px_req[r*NCOL + c], held high until acknowledged
  input  wire [NPX-1:0]  px_req,
  output wire [NPX-1:0]  px_ack,

  output wire            out_req,
  output wire [AW-1:0]   out_addr,            // {row, col}
  input  wire            out_ack
);

  //--------------------------------------------------------------------
  // 1. row activity  : wired-OR of each row's pixels
  //--------------------------------------------------------------------
  wire [NROW-1:0] row_req;
  genvar r, c;
  generate
    for (r = 0; r < NROW; r = r + 1) begin : g_rowor
      assign row_req[r] = |px_req[r*NCOL +: NCOL];
    end
  endgenerate

  //--------------------------------------------------------------------
  // 2. row arbitration
  //--------------------------------------------------------------------
  wire [NROW-1:0]  row_g;
  wire [ROW_D-1:0] row_addr;
  wire             row_any;

  arb_tree #(.DEPTH(ROW_D), .M_TPD(M_TPD)) u_rowarb (
    .req   (row_req),
    .gin   (1'b1),
    .rup   (row_any),
    .grant (row_g),
    .addr  (row_addr)
  );

  //--------------------------------------------------------------------
  // 3. shared column lines, driven only by the granted row
  //
  //    col_req[c] = OR over r of ( row_g[r] & px_req[r][c] )
  //    In silicon this is the shared column wire; only the selected
  //    row's pixels pull it. That sharing IS the area saving, and it is
  //    also why the column tree is a single bottleneck resource.
  //--------------------------------------------------------------------
  reg [NCOL-1:0] col_req;
  integer rr, cc;
  always @(*) begin
    for (cc = 0; cc < NCOL; cc = cc + 1) begin
      col_req[cc] = 1'b0;
      for (rr = 0; rr < NROW; rr = rr + 1)
        col_req[cc] = col_req[cc] | (row_g[rr] & px_req[rr*NCOL + cc]);
    end
  end

  //--------------------------------------------------------------------
  // 4. column arbitration
  //--------------------------------------------------------------------
  wire [NCOL-1:0]  col_g;
  wire [COL_D-1:0] col_addr;
  wire             col_any;

  arb_tree #(.DEPTH(COL_D), .M_TPD(M_TPD)) u_colarb (
    .req   (col_req),
    .gin   (1'b1),
    .rup   (col_any),
    .grant (col_g),
    .addr  (col_addr)
  );

  //--------------------------------------------------------------------
  // 5. output stage : same control as aer_node
  //--------------------------------------------------------------------
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
  wire granted = (|row_g) & (|col_g);

  wire cin, cout;
  wire cout_n  = ~cout;
  wire r_ack_n = ~out_ack;
  wire cin_en  = cout_n & r_ack_n;

  c_element #(.TPD_NS(C_TPD)) u_cin  (.rst_n(rst_n), .a(granted), .b(cin_en),  .z(cin));
  c_element #(.TPD_NS(C_TPD)) u_cout (.rst_n(rst_n), .a(cin),     .b(r_ack_n), .z(cout));

  // acknowledge exactly the granted pixel
  generate
    for (r = 0; r < NROW; r = r + 1) begin : g_ackr
      for (c = 0; c < NCOL; c = c + 1) begin : g_ackc
        assign px_ack[r*NCOL + c] = row_g[r] & col_g[c] & cin;
      end
    end
  endgenerate

  // address FF, clocked by the locally generated acknowledge
  reg [AW-1:0] addr_q;
  always @(posedge cin or negedge rst_n) begin
    if (!rst_n) addr_q <= {AW{1'b0}};
    else        addr_q <= {row_addr, col_addr};
  end
  assign out_addr = addr_q;

  delay_chain #(.N(REQ_DLY)) u_req_dly (.in(cout), .out(out_req));

endmodule
