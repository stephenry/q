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
// Enqueue
  input wire logic                                   i_push
// Dequeue
, input wire logic                                   i_pop

, output wire logic                                  o_wen
, output wire logic [ADDR_W - 1:0]                   o_wa

, output wire logic                                  o_ren
, output wire logic [ADDR_W - 1:0]                   o_ra

// -------------------------------------------------------------------------- //
// Status:
, output wire logic                                  o_full_w
, output wire logic                                  o_empty_w

// -------------------------------------------------------------------------- //
//
, input wire logic                                   clk
, input wire logic                                   arst_n
);

`Q_DFFRE(logic [ADDR_W:0], ra, 'b0, clk);
`Q_DFFRE(logic [ADDR_W:0], wa, 'b0, clk);

// ========================================================================== //
//                                                                            //
// Logic                                                                      //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign wa_w = i_push ? (wa_r + 'b1) : wa_r;

assign ra_w = i_pop ? (ra_r + 'b1) : ra_r;

// -------------------------------------------------------------------------- //
//
assign o_full_w = (ra_w [ADDR_W] ^ wa_w [ADDR_W]) &
                  (ra_w [ADDR_W - 1:0] == wa_w [ADDR_W - 1:0]);

// -------------------------------------------------------------------------- //
//
assign o_empty_w = (ra_w == wa_w);

// ========================================================================== //
//                                                                            //
// Outputs                                                                    //
//                                                                            //
// ========================================================================== //

assign o_wen = i_push;
assign o_ren = i_pop;

assign o_wa = wa_r;
assign o_ra = ra_r;

endmodule : queue_cntrl

`include "unmacros.vh"
