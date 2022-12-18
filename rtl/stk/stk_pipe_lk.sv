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

module stk_pipe_lk (
// -------------------------------------------------------------------------- //
//
  input wire logic                                i_lk_vld_r
, input wire stk_pkg::engid_t                     i_lk_engid_r
, input wire stk_pkg::opcode_t                    i_lk_opcode_r
, input wire logic                                i_lk_dat_vld_r
, input wire logic [127:0]                        i_lk_dat_r

// -------------------------------------------------------------------------- //
// Allocation Interface
, input wire stk_pkg::ptr_t                       i_lk_ptr

// -------------------------------------------------------------------------- //
// Clk/Reset
, input wire logic                                clk
, input wire logic                                arst_n
);

// Stack empty flag status on a per. engine basis.
`Q_DFFR(logic [cfg_pkg::ENGS_N - 1:0], empty, 'b0, clk);

// -------------------------------------------------------------------------- //
//
rf #(.W(stk_pkg::PTR_W), .N(cfg_pkg::ENGS_N)) u_rf_head (
//
  .i_ra                       ()
, .o_rdata                    ()
//
, .i_wen                      ()
, .i_wa                       ()
, .i_wdata                    ()
//
, .clk                        (clk)
);

// -------------------------------------------------------------------------- //
//
rf #(.W(stk_pkg::PTR_W), .N(cfg_pkg::ENGS_N)) u_rf_tail (
//
  .i_ra                       ()
, .o_rdata                    ()
//
, .i_wen                      ()
, .i_wa                       ()
, .i_wdata                    ()
//
, .clk                        (clk)
);

endmodule : stk_pipe_lk

`include "unmacros.vh"
