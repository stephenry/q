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
`include "stk/stk_pkg.vh"

module stk_pipe_mem (

// -------------------------------------------------------------------------- //
// Head SRAM interfaces
  input wire logic [stk_pkg::BANKS_N - 1:0]       i_lk_ptr_head_ce
, input wire logic [stk_pkg::BANKS_N - 1:0]       i_lk_ptr_head_oe
, input wire stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]
                                                  i_lk_ptr_head_addr
, input wire stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]
                                                  i_lk_ptr_head_din
//
, output wire stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]
                                                  o_lk_ptr_head_dout

// -------------------------------------------------------------------------- //
// Tail SRAM interfaces
, input wire logic [stk_pkg::BANKS_N - 1:0]       i_lk_ptr_tail_ce
, input wire logic [stk_pkg::BANKS_N - 1:0]       i_lk_ptr_tail_oe
, input wire stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]
                                                  i_lk_ptr_tail_addr
, input wire stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]
                                                  i_lk_ptr_tail_din
//
, output wire stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]
                                                  o_lk_ptr_tail_dout

// -------------------------------------------------------------------------- //
// Data SRAM interfaces
, input wire logic [stk_pkg::BANKS_N - 1:0]       i_lk_ptr_dat_ce
, input wire logic [stk_pkg::BANKS_N - 1:0]       i_lk_ptr_dat_oe
, input wire stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]
                                                  i_lk_ptr_dat_addr
, input wire [stk_pkg::BANKS_N - 1:0][127:0]
                                                  i_lk_ptr_dat_din
//
, output wire logic [stk_pkg::BANKS_N - 1:0][127:0]
                                                  o_lk_ptr_dat_dout

// -------------------------------------------------------------------------- //
// Clk/Reset
, input wire logic                                clk
);


// ========================================================================== //
//                                                                            //
//  Head Pointer SRAM                                                         //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
for (genvar bnk = 0; bnk < stk_pkg::BANKS_N; bnk++) begin : head_sram_GEN

stk_pipe_mem_head_sram u_stk_pipe_mem_head_sram (
//
  .i_addr                     (i_lk_ptr_head_addr [bnk])
, .i_din                      (i_lk_ptr_head_din [bnk])
, .i_ce                       (i_lk_ptr_head_ce [bnk])
, .i_oe                       (i_lk_ptr_head_oe [bnk])
//
, .o_dout                     (o_lk_ptr_head_dout [bnk])
//
, .clk                        (clk)
);

end : head_sram_GEN

// ========================================================================== //
//                                                                            //
//  Tail Pointer SRAM                                                         //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
for (genvar bnk = 0; bnk < stk_pkg::BANKS_N; bnk++) begin : tail_sram_GEN

stk_pipe_mem_tail_sram u_stk_pipe_mem_tail_sram (
//
  .i_addr                     (i_lk_ptr_tail_addr [bnk])
, .i_din                      (i_lk_ptr_tail_din [bnk])
, .i_ce                       (i_lk_ptr_tail_ce [bnk])
, .i_oe                       (i_lk_ptr_tail_oe [bnk])
//
, .o_dout                     (o_lk_ptr_tail_dout [bnk])
//
, .clk                        (clk)
);

end : tail_sram_GEN

// ========================================================================== //
//                                                                            //
//  Data Pointer SRAM                                                         //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
for (genvar bnk = 0; bnk < stk_pkg::BANKS_N; bnk++) begin : data_sram_GEN

stk_pipe_mem_data_sram u_stk_pipe_mem_data_sram (
//
  .i_addr                     (i_lk_ptr_dat_addr [bnk])
, .i_din                      (i_lk_ptr_dat_din [bnk])
, .i_ce                       (i_lk_ptr_dat_ce [bnk])
, .i_oe                       (i_lk_ptr_dat_oe [bnk])
//
, .o_dout                     (o_lk_ptr_dat_dout [bnk])
//
, .clk                        (clk)
);

end : data_sram_GEN

endmodule : stk_pipe_mem

`include "unmacros.vh"