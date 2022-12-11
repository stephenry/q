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

module queue_cntrl #(
  // Total number of entries; (constrained to be a power-of-2)
  parameter int            N

, parameter int            ADDR_W = $clog2(N)
) (
// -------------------------------------------------------------------------- //
// Dequeue
  input wire logic                                   i_push
// Enqueue
, input wire logic                                   i_pop

, output wire logic                                  o_wr_en
, output wire logic [ADDR_W - 1:0]                   o_wr_addr

, output wire logic                                  o_rd_en
, output wire logic [ADDR_W - 1:0]                   o_rd_addr

// -------------------------------------------------------------------------- //
// Status:
, output wire logic                                  o_full_w
, output wire logic                                  o_empty_w

// -------------------------------------------------------------------------- //
//
, input wire logic                                   clk
, input wire logic                                   arst_n
);

`Q_DFFENR(logic [ADDR_W:0], rd_addr, 'b0);
`Q_DFFENR(logic [ADDR_W:0], wr_addr, 'b0);

// ========================================================================== //
//                                                                            //
// Logic                                                                      //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign wr_addr_w = i_push ? (wr_addr_r + 'b1) : wr_addr_r;

assign rd_addr_w = i_pop ? (rd_addr_r + 'b1) : rd_addr_r;

// -------------------------------------------------------------------------- //
//
assign o_full_w = (rd_addr_w [ADDR_W] ^ wr_addr_w [ADDR_W]) &
                  (rd_addr_w [ADDR_W - 1:0] == wr_addr_w [ADDR_W - 1:0]);

// -------------------------------------------------------------------------- //
//
assign o_empty_w = (rd_addr_w == wr_addr_w);

// ========================================================================== //
//                                                                            //
// Outputs                                                                    //
//                                                                            //
// ========================================================================== //

assign o_wr_en = i_push;
assign o_rd_en = i_pop;

assign o_wr_addr = wr_addr_r;
assign o_rd_addr = rd_addr_r;

endmodule : queue_cntrl

`include "unmacros.vh"
