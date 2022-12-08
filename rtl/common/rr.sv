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
`include "macros.vh"

module rr #(
// Arbiter width
  parameter int              W
) (
// -------------------------------------------------------------------------- //
//
  input wire logic [W - 1:0]                         i_req
, input wire logic                                   i_ack
, output wire logic [W - 1:0]                        o_gnt

// -------------------------------------------------------------------------- //
//
, input wire logic                                   clk
, input wire logic                                   arst_n
);

localparam int PTR_W = $clog2(W);

`Q_DFFENR(logic [PTR_W - 1:0], ptr, 'b0);
logic [W - 1:0] ptr_dec;
logic [W - 1:0] mask_lsb;
logic [W - 1:0] req_masked_lsb;
logic [W - 1:0] mask_msb;
logic [W - 1:0] req_masked_msb;
logic [W - 1:0] sel_lsb;
logic [W - 1:0] sel_msb;
logic [W - 1:0] gnt_nxt;
logic [W - 1:0] gnt_nxt_enc;

// ========================================================================== //
//                                                                            //
// Logic                                                                      //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign ptr_w = i_ack ? gnt_nxt_enc : ptr_r;

// -------------------------------------------------------------------------- //
//
dec #(.W) u_ptr_dec (.i_x(ptr_r), .o_y(ptr_dec));

// -------------------------------------------------------------------------- //
// Compute LSB-oriented selection bitmap
mask #(.W, .TOWARDS_LSB(1'b1)) u_left_lsb (
, .i_x(ptr_dec), .o_y(mask_lsb)
);

assign req_masked_lsb = i_req & mask_lsb;

pri #(.W, .FROM_LSB(1'b1)) u_sel_lsb_pri (
, .i_x(req_masked_lsb), .o_y(sel_lsb)
);

// -------------------------------------------------------------------------- //
// Compute MSB-oriented selection bitmap
mask #(.W, .TOWARDS_LSB(1'b0), .INCLUSIVE(1'b1)) u_left_msb (
, .i_x(ptr_dec), .o_y(mask_msb)
);

assign req_masked_msb = i_req & mask_msb;

pri #(.W, .FROM_LSB(1'b1)) u_sel_msb_pri (
, .i_x(req_masked_msb), .o_y(sel_msb)
);

// -------------------------------------------------------------------------- //
// Form decoded GNT signal; if MSB oriented vector is non-zero, otherwise
// select LSB oriented vector to consider any other remaining requestors.
assign gnt = (sel_msb != '0) ? sel_msb : sel_lsb;

// Advance search point based upon current grant.
assign gnt_nxt = { gnt [W - 2:0], gnt [W - 1] };

// Encode grant next to form next search pointer.
enc #(.W) u_gnt_enc (.i_x(gnt_nxt), .o_y(gnt_nxt_enc));

// ========================================================================== //
//                                                                            //
// Outputs                                                                    //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign o_gnt = gnt;

endmodule : rr

`include "unmacros.vh"