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

`include "q_pkg.vh"
`include "cfg_pkg.vh"
`include "stk/stk_pkg.vh"

module stk_pipe (
// -------------------------------------------------------------------------- //
  input wire stk_pkg::opcode_t [cfg_pkg::ENGS_N - 1:0]
                                                  i_cmd_opcode
, input wire logic [cfg_pkg::ENGS_N - 1:0][127:0] i_cmd_dat
//
, output wire logic [cfg_pkg::ENGS_N - 1:0]       o_cmd_ack

// -------------------------------------------------------------------------- //
//
, output wire logic [cfg_pkg::ENGS_N - 1:0]       o_rsp_vld

// -------------------------------------------------------------------------- //
// Clk/Reset
, input wire logic                                clk
, input wire logic                                arst_n
);

// -------------------------------------------------------------------------- //
// AD <-> AL
logic                         al_ad_empty_r;
logic                         al_ad_busy;
logic                         ad_al_alloc;

// -------------------------------------------------------------------------- //
// AD -> LK
`Q_DFFR(logic, lk_vld, 1'b0, clk);
`Q_DFFE(stk_pkg::engid_t, lk_engid, lk_vld_w, clk);
`Q_DFFE(stk_pkg::opcode_t, lk_opcode, lk_vld_w, clk);
`Q_DFFE(logic, lk_dat_vld, lk_vld_w, clk);
logic lk_dat_en;
`Q_DFFE(logic [127:0], lk_dat, lk_dat_en, clk);

// -------------------------------------------------------------------------- //
// AL -> LK
stk_pkg::ptr_t                al_lk_ptr;

// -------------------------------------------------------------------------- //
// LK -> ?

// -------------------------------------------------------------------------- //
//
stk_pipe_ad u_stk_pipe_ad (
//
  .i_cmd_opcode               (i_cmd_opcode)
, .i_cmd_dat                  (i_cmd_dat)
, .o_cmd_ack                  (o_cmd_ack)
//
, .o_lk_vld_w                 (lk_vld_w)
, .o_lk_engid_w               (lk_engid_w)
, .o_lk_opcode_w              (lk_opcode_w)
, .o_lk_dat_vld_w             (lk_dat_vld_w)
, .o_lk_dat_w                 (lk_dat_w)
//
, .i_al_empty_r               (al_ad_empty_r)
, .i_al_busy                  (al_ad_busy)
, .o_al_alloc                 (ad_al_alloc)
//
, .i_rsp_vld                  (o_rsp_vld)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

assign lk_dat_en = (lk_vld_w & lk_dat_vld_w);

// -------------------------------------------------------------------------- //
//
stk_pipe_lk u_stk_pipe_lk (
//
  .i_lk_vld_r                 (lk_vld_r)
, .i_lk_engid_r               (lk_engid_r)
, .i_lk_opcode_r              (lk_opcode_r)
, .i_lk_dat_vld_r             (lk_dat_vld_r)
, .i_lk_dat_r                 (lk_dat_r)
//
, .i_lk_ptr                   (al_lk_ptr)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

// -------------------------------------------------------------------------- //
//
stk_pipe_al u_stk_pipe_al (
//
  .i_ad_alloc                 (ad_al_alloc)
, .o_ad_empty_r               (al_ad_empty_r)
, .o_ad_busy                  (al_ad_busy)
//
, .o_lk_ptr_w                 (al_lk_ptr)
//
, .i_dealloc_vld              ()
, .i_dealloc_ptr              ()
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

// -------------------------------------------------------------------------- //
//
stk_pipe_wb u_stk_pipe_wb (
//
  .clk                        (clk)
, .arst_n                     (arst_n)
);

endmodule : stk_pipe
