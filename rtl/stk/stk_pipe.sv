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

// ========================================================================== //
//                                                                            //
//  Wires                                                                     //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// AD <-> AL
logic                         al_ad_empty_r;
logic                         al_ad_busy_r;
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
// LK -> MEM
//
`Q_DFFR(logic, mem_uc_vld, 1'b0, clk);
`Q_DFFE(stk_pkg::engid_t, mem_uc_engid, mem_uc_vld_w, clk);
`Q_DFFE(stk_pkg::bank_id_t, mem_uc_bankid, mem_uc_vld_w, clk);
`Q_DFFE(logic, mem_uc_head_vld, mem_uc_vld_w, clk);
`Q_DFFE(stk_pkg::ptr_t, mem_uc_head_ptr, mem_uc_vld_w, clk);
`Q_DFFE(logic, mem_uc_tail_vld, mem_uc_vld_w, clk);
`Q_DFFE(stk_pkg::ptr_t, mem_uc_tail_ptr, mem_uc_vld_w, clk);
//
`Q_DFF(logic [stk_pkg::BANKS_N- 1:0], lk_prev_ptr_ce, clk);
`Q_DFF(logic [stk_pkg::BANKS_N- 1:0], lk_prev_ptr_oe, clk);
`Q_DFF(stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0], lk_prev_ptr_addr, clk);
`Q_DFF(stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0], lk_prev_ptr_din, clk);
`Q_DFF(stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0], lk_prev_ptr_dout, clk);
//
`Q_DFF(logic [stk_pkg::BANKS_N- 1:0], lk_ptr_dat_ce, clk);
`Q_DFF(logic [stk_pkg::BANKS_N- 1:0], lk_ptr_dat_oe, clk);
`Q_DFF(stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0], lk_ptr_dat_addr, clk);
`Q_DFF(logic [stk_pkg::BANKS_N - 1:0][127:0], lk_ptr_dat_din, clk);
`Q_DFF(logic [stk_pkg::BANKS_N - 1:0][127:0], lk_ptr_dat_dout, clk);

// -------------------------------------------------------------------------- //
// MEM -> WRBK
//
`Q_DFFR(logic, wrbk_uc_vld, 1'b0, clk);
`Q_DFFE(stk_pkg::engid_t, wrbk_uc_engid, wrbk_uc_vld_w, clk);
`Q_DFFE(stk_pkg::bank_id_t, wrbk_uc_bankid, wrbk_uc_vld_w, clk);
`Q_DFFE(logic, wrbk_uc_head_vld, wrbk_uc_vld_w, clk);
`Q_DFFE(stk_pkg::ptr_t, wrbk_uc_head_ptr, wrbk_uc_vld_w, clk);
`Q_DFFE(logic, wrbk_uc_tail_vld, wrbk_uc_vld_w, clk);
`Q_DFFE(stk_pkg::ptr_t, wrbk_uc_tail_ptr, wrbk_uc_vld_w, clk);

// ========================================================================== //
//                                                                            //
//  Admission (AD) Stage                                                      //
//                                                                            //
// ========================================================================== //

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
, .i_al_busy_r                (al_ad_busy_r)
, .o_al_alloc                 (ad_al_alloc)
//
, .i_wrbk_uc_vld_r            (wrbk_uc_vld_r)
, .i_wrbk_uc_engid_r          (wrbk_uc_engid_r)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

// ========================================================================== //
//                                                                            //
//  Allocation (AL) Stage                                                     //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
stk_pipe_al u_stk_pipe_al (
//
  .i_ad_alloc                 (ad_al_alloc)
, .o_ad_empty_r               (al_ad_empty_r)
, .o_ad_busy_r                (al_ad_busy_r)
//
, .o_lk_ptr_w                 (al_lk_ptr)
//
, .i_dealloc_vld              ()
, .i_dealloc_ptr              ()
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

assign lk_dat_en = (lk_vld_w & lk_dat_vld_w);

// ========================================================================== //
//                                                                            //
//  Lookup (LK) Stage                                                         //
//                                                                            //
// ========================================================================== //

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
, .i_wrbk_uc_vld_r            (wrbk_uc_vld_r)
, .i_wrbk_uc_engid_r          (wrbk_uc_engid_r)
, .i_wrbk_uc_head_vld_r       (wrbk_uc_head_vld_r)
, .i_wrbk_uc_head_ptr_r       (wrbk_uc_head_ptr_r)
, .i_wrbk_uc_tail_vld_r       (wrbk_uc_tail_vld_r)
, .i_wrbk_uc_tail_ptr_r       (wrbk_uc_tail_ptr_r)
//
, .o_lk_prev_ptr_ce_w         (lk_prev_ptr_ce_w)
, .o_lk_prev_ptr_oe_w         (lk_prev_ptr_oe_w)
, .o_lk_prev_ptr_addr_w       (lk_prev_ptr_addr_w)
, .o_lk_prev_ptr_din_w        (lk_prev_ptr_din_w)
//
, .o_lk_ptr_dat_ce_w          (lk_ptr_dat_ce_w)
, .o_lk_ptr_dat_oe_w          (lk_ptr_dat_oe_w)
, .o_lk_ptr_dat_addr_w        (lk_ptr_dat_addr_w)
, .o_lk_ptr_dat_din_w         (lk_ptr_dat_din_w)
//
, .o_mem_uc_vld_w             (mem_uc_vld_w)
, .o_mem_uc_engid_w           (mem_uc_engid_w)
, .o_mem_uc_bankid_w          (mem_uc_bankid_w)
, .o_mem_uc_head_vld_w        (mem_uc_head_vld_w)
, .o_mem_uc_head_ptr_w        (mem_uc_head_ptr_w)
, .o_mem_uc_tail_vld_w        (mem_uc_tail_vld_w)
, .o_mem_uc_tail_ptr_w        (mem_uc_tail_ptr_w)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

// ========================================================================== //
//                                                                            //
//  Memory (MEM) Stage                                                        //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
stk_pipe_mem u_stk_pipe_mem (
//
  .i_lk_prev_ptr_ce_r         (lk_prev_ptr_ce_r)
, .i_lk_prev_ptr_oe_r         (lk_prev_ptr_oe_r)
, .i_lk_prev_ptr_addr_r       (lk_prev_ptr_addr_r)
, .i_lk_prev_ptr_din_r        (lk_prev_ptr_din_r)
, .o_lk_prev_ptr_dout_w       (lk_prev_ptr_dout_w)
//
, .i_lk_ptr_dat_ce_r          (lk_ptr_dat_ce_r)
, .i_lk_ptr_dat_oe_r          (lk_ptr_dat_oe_r)
, .i_lk_ptr_dat_addr_r        (lk_ptr_dat_addr_r)
, .i_lk_ptr_dat_din_r         (lk_ptr_dat_din_r)
, .o_lk_ptr_dat_dout_w        (lk_ptr_dat_dout_w)
//
, .i_mem_uc_vld_r             (mem_uc_vld_r)
, .i_mem_uc_engid_r           (mem_uc_engid_r)
, .i_mem_uc_bankid_r          (mem_uc_bankid_r)
, .i_mem_uc_head_vld_r        (mem_uc_head_vld_r)
, .i_mem_uc_head_ptr_r        (mem_uc_head_ptr_r)
, .i_mem_uc_tail_vld_r        (mem_uc_tail_vld_r)
, .i_mem_uc_tail_ptr_r        (mem_uc_tail_ptr_r)
//
, .o_wrbk_uc_vld_w            (wrbk_uc_vld_w)
, .o_wrbk_uc_engid_w          (wrbk_uc_engid_w)
, .o_wrbk_uc_head_vld_w       (wrbk_uc_head_vld_w)
, .o_wrbk_uc_head_ptr_w       (wrbk_uc_head_ptr_w)
, .o_wrbk_uc_tail_vld_w       (wrbk_uc_tail_vld_w)
, .o_wrbk_uc_tail_ptr_w       (wrbk_uc_tail_ptr_w)
//
, .clk                        (clk)
);

endmodule : stk_pipe
