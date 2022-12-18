//========================================================================== //
// Copyright (c) 2022, Stephen Henry
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//========================================================================== //

`include "common_defs.vh"

module rf #(
// Word width (bits)
//
  parameter int                         W

// Word count
//
, parameter int                         N

// Read port count
//
, parameter int                         RD_N  = 1

// Write port count
//
, parameter int                         WR_N  = 1
) (

// -------------------------------------------------------------------------- //
// Read ports
  input wire logic [RD_N - 1:0][$clog2(N) - 1:0]      i_ra
//
, output wire logic [RD_N - 1:0][W - 1:0]             o_rdata

// -------------------------------------------------------------------------- //
// Write ports
, input wire logic [RD_N - 1:0]                       i_wen
, input wire logic [RD_N - 1:0][$clog2(N) - 1:0]      i_wa
, input wire logic [RD_N - 1:0][W - 1:0]              i_wdata

// -------------------------------------------------------------------------- //
//
, input wire logic                                   clk
);

// Write port signals:
logic [WR_N - 1:0][N - 1:0]            wr_sel;
logic [N - 1:0][WR_N - 1:0]            wr_port_sel;

// Read port signals:
logic [RD_N - 1:0][N - 1:0]            rd_sel;
logic [N - 1:0][W - 1:0]               rdata;

logic [N - 1:0]                        flop_clk;
logic [N - 1:0][W - 1:0]               flop_w;
logic [N - 1:0][W - 1:0]               flop_r;
logic [N - 1:0]                        flop_en;

// ========================================================================== //
//                                                                            //
//  Write ports                                                               //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
for (genvar wr = 0; wr < WR_N; wr++) begin : write_dec_GEN

dec #(.N(WR_N), .W) u_wr_dec (.o_y(wr_sel [wr]), .i_x(i_wa [wr]));

end : write_dec_GEN

// -------------------------------------------------------------------------- //
//
for (genvar n = 0; n < N; n++) begin : write_sel_word_GEN

for (genvar wr = 0; wr < WR_N; wr++) begin : write_sel_port_GEN

assign wr_port_sel [n][wr] = (i_wen [wr] & ra_d [wr][n]);

end : write_sel_port_GEN

end : write_sel_word_GEN

// -------------------------------------------------------------------------- //
//
for (genvar n = 0; n < N; n++) begin : write_GEN

mux #(.N(WR_N), .W) u_mux (
  .i_x(i_wdata), .i_sel(wr_port_sel [n]), .o_y(flop_w [n])
);

assign flop_en [j] = (wr_port_sel [n] != '0);

end : write_GEN

// ========================================================================== //
//                                                                            //
//  Register-File state                                                       //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
for (genvar n = 0; n < N; n++) begin : flop_GEN

// ICG for word 'N'
// TODO(stephenry): swap out dffen for dff when icg is enabled.
icg u_wordn_icg (
  .o_clk_gated(flop_clk [n]), .i_clk(clk), .i_en(flop_en [n]), .i_dft_en(1'b0)
);

// RF state word 'N'
dffen #(.W) u_wordn_dffen (
  .d(flop_w[n]), .en(flop_en[n]), .q(flop_r[n]), .clk(flop_clk [n])
);

end : flop_GEN

// ========================================================================== //
//                                                                            //
//  Read ports                                                                //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
for (genvar rd = 0; rd < RD_PORTS_N; rd++) begin : read_GEN

// Read address decoder
dec #(.N) u_rd_dec (.o_y(rd_sel [rd]), .i_x(i_ra [rd]));

// Read word-select mux.
mux #(.N, .W) u_rd_mux (
  .i_x(flop_r), .i_sel(rd_sel [rd]), .o_y(rdata [rd])
);

end : read_GEN

// ========================================================================== //
//                                                                            //
// Outputs                                                                    //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign o_rdata = rd_data;

endmodule : rf
