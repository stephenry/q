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

module stack_cntrl #(
// Stack entries
  parameter int              N

, parameter int              ADDR_W = $clog2(N)
) (
// -------------------------------------------------------------------------- //
// Control
  input wire logic                                   i_push
, input wire logic                                   i_pop

, output wire logic                                  o_mem_wen
, output wire logic                                  o_mem_ren
, output wire logic [ADDR_W - 1:0]                   o_mem_addr

// -------------------------------------------------------------------------- //
// Status:
, output wire logic                                  o_full_w
, output wire logic                                  o_empty_w

// -------------------------------------------------------------------------- //
//
, input wire logic                                   clk
, input wire logic                                   arst_n
);

localparam int FIRST_ENTRY = 0;
localparam int LAST_ENTRY = (N - 1);

logic                        empty_en;
`Q_DFFRE(logic, empty, empty_en, 1'b1, clk);
logic                        full_en;
`Q_DFFRE(logic, full, full_en, 1'b0, clk);
logic                        ptr_en;
`Q_DFFRE(logic [ADDR_W - 1:0], ptr, ptr_en, 'b0, clk);
logic                        state_upt;

// ========================================================================== //
//                                                                            //
// Logic                                                                      //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Advance internal controller state on push or pop operation.
//
assign state_upt = (i_push | i_pop);

// -------------------------------------------------------------------------- //
// Empty cleared on push on empty (1); set on pop from zeroth entry (2).
//
assign empty_en = state_upt;
assign empty_w = (empty_r & (~i_push))                                // (1)
               | (i_pop & (ptr_r == FIRST_ENTRY[ADDR_W - 1:0]));      // (2)

// -------------------------------------------------------------------------- //
// Full cleared on pop from full state (1); set on push to N'th (final)
// entry (2).
//
assign full_en = state_upt;
assign full_w = (full_r & (~i_pop))                                  // (1)
              | (i_push & (ptr_r == LAST_ENTRY[ADDR_W - 1:0]));      // (2)

// -------------------------------------------------------------------------- //
//
assign ptr_en = state_upt;
assign ptr_w = (i_push & ~full_w) ? (ptr_r + 'b1) :
               (i_pop & ~empty_w) ? (ptr_r - 'b1) :
               ptr_r;

// ========================================================================== //
//                                                                            //
// Outputs                                                                    //
//                                                                            //
// ========================================================================== //

assign o_mem_wen = i_push;
assign o_mem_ren = i_pop;
assign o_mem_addr = ptr_r;

assign o_empty_w = empty_w;
assign o_full_w = full_w;

endmodule : stack_cntrl

`include "unmacros.vh"
