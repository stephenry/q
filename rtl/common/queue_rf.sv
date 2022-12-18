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

module queue_rf #(
  // Total number of entries; (constrained to be a power-of-2)
  parameter int            N
  // The width of each entry
, parameter int            W
) (
// -------------------------------------------------------------------------- //
// Enqueue
  input wire logic                                   i_push
, input wire logic [W - 1:0]                         i_push_dat

// -------------------------------------------------------------------------- //
// Dequeue
, input wire logic                                   i_pop
, output wire logic [W - 1:0]                        o_pop_dat

// -------------------------------------------------------------------------- //
// Status:
, output wire logic                                  o_full_w
, output wire logic                                  o_empty_w

// -------------------------------------------------------------------------- //
//
, input wire logic                                   clk
, input wire logic                                   arst_n
);

localparam int ADDR_W = $clog2(N);

logic [ADDR_W - 1:0] ra;

logic [ADDR_W - 1:0] wa;
logic                wen;


// ========================================================================== //
//                                                                            //
// Instance                                                                   //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
rf #(.W, .N) u_rf (
//
  .i_ra                       (ra)
, .o_rdata                    (o_pop_dat)
//
, .i_wen                      (wen)
, .i_wa                       (wa)
, .i_wdata                    (i_push_dat)
//
, .clk                        (clk)
);

// -------------------------------------------------------------------------- //
//
queue_cntrl #(.N, .ADDR_W) u_queue_cntrl (
//
  .i_push                     (i_push)
, .i_pop                      (i_pop)
//
, .o_wen                      (wen)
, .o_wa                       (wa)
//
, .o_ren                      ()
, .o_ra                       (ra)
//
, .o_full_w                   (o_full_w)
, .o_empty_w                  (o_empty_w)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

endmodule : queue_rf

`include "unmacros.vh"
