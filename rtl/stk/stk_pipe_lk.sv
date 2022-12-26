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
// MEM microcode update (reg).
, input wire logic                                i_wrbk_uc_vld_r
, input wire stk_pkg::engid_t                     i_wrbk_uc_engid_r
, input wire logic                                i_wrbk_uc_set_empty_r
, input wire logic                                i_wrbk_uc_clr_empty_r
, input wire logic                                i_wrbk_uc_head_vld_r
, input wire stk_pkg::ptr_t                       i_wrbk_uc_head_ptr_r
, input wire logic                                i_wrbk_uc_tail_vld_r
, input wire stk_pkg::ptr_t                       i_wrbk_uc_tail_ptr_r
//

// -------------------------------------------------------------------------- //
// Tail SRAM interfaces
, output wire logic [stk_pkg::BANKS_N - 1:0]      o_lk_prev_ptr_ce
, output wire logic [stk_pkg::BANKS_N - 1:0]      o_lk_prev_ptr_oe
, output wire stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]
                                                  o_lk_prev_ptr_addr
, output wire stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]
                                                  o_lk_prev_ptr_din

// -------------------------------------------------------------------------- //
// Data SRAM interfaces
, output wire logic [stk_pkg::BANKS_N - 1:0]      o_lk_ptr_dat_ce
, output wire logic [stk_pkg::BANKS_N - 1:0]      o_lk_ptr_dat_oe
, output wire stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]
                                                  o_lk_ptr_dat_addr
, output wire [stk_pkg::BANKS_N - 1:0][127:0]
                                                  o_lk_ptr_dat_din

// -------------------------------------------------------------------------- //
// MEM microcode update (nxt).
, output wire logic                               o_mem_uc_vld_w
, output wire stk_pkg::engid_t                    o_mem_uc_engid_w
, output wire stk_pkg::bank_id_t                  o_mem_uc_bankid_w
, output wire logic                               o_mem_uc_set_empty_w
, output wire logic                               o_mem_uc_clr_empty_w
, output wire logic                               o_mem_uc_head_vld_w
, output wire stk_pkg::ptr_t                      o_mem_uc_head_ptr_w
, output wire logic                               o_mem_uc_tail_vld_w
, output wire stk_pkg::ptr_t                      o_mem_uc_tail_ptr_w

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
// Decoder:
//
logic                                   cmd_vld;
logic                                   cmd_is_push;
logic                                   cmd_is_pop;
logic                                   cmd_is_inv;

//
logic                                   mem_uc_vld;
stk_pkg::engid_t                        mem_uc_engid;
logic                                   mem_uc_set_empty;
logic                                   mem_uc_clr_empty;
logic                                   mem_uc_head_vld;
stk_pkg::ptr_t                          mem_uc_head_ptr;
logic                                   mem_uc_tail_vld;
stk_pkg::ptr_t                          mem_uc_tail_ptr;

//
logic                                   rf_head_wen;
stk_pkg::engid_t                        rf_head_wa;
stk_pkg::ptr_t                          rf_head_wdata;
stk_pkg::engid_t                        rf_head_ra;
stk_pkg::ptr_t                          rf_head_rdata;

//
logic                                   rf_tail_wen;
stk_pkg::engid_t                        rf_tail_wa;
stk_pkg::ptr_t                          rf_tail_wdata;
stk_pkg::engid_t                        rf_tail_ra;
stk_pkg::ptr_t                          rf_tail_rdata;

// Empty Status
//
logic                                   engid_is_empty;
`Q_DFFR(logic [cfg_pkg::ENGS_N - 1:0], empty, '1, clk);

// ========================================================================== //
//                                                                            //
//  Decoder                                                                   //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Command decoder
assign cmd_vld = (i_lk_opcode_r != stk_pkg::OPCODE_NOP);
assign cmd_is_push = (i_lk_opcode_r == stk_pkg::OPCODE_PUSH);
assign cmd_is_pop = (i_lk_opcode_r == stk_pkg::OPCODE_POP);
assign cmd_is_inv = (i_lk_opcode_r == stk_pkg::OPCODE_INV);

// ========================================================================== //
//                                                                            //
//  Head Pointer Table                                                        //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign rf_head_ra = i_lk_engid_r;

// -------------------------------------------------------------------------- //
//
rf #(.W(stk_pkg::PTR_W), .N(cfg_pkg::ENGS_N)) u_rf_head (
//
  .i_ra                       (rf_head_ra)
, .o_rdata                    (rf_head_rdata)
//
, .i_wen                      (rf_head_wen)
, .i_wa                       (rf_head_wa)
, .i_wdata                    (rf_head_wdata)
//
, .clk                        (clk)
);

// ========================================================================== //
//                                                                            //
//  Tail Pointer Table                                                        //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign rf_tail_ra = i_lk_engid_r;

// -------------------------------------------------------------------------- //
//
rf #(.W(stk_pkg::PTR_W), .N(cfg_pkg::ENGS_N)) u_rf_tail (
//
  .i_ra                       (rf_tail_ra)
, .o_rdata                    (rf_tail_rdata)
//
, .i_wen                      (rf_tail_wen)
, .i_wa                       (rf_tail_wa)
, .i_wdata                    (rf_tail_wdata)
//
, .clk                        (clk)
);

// ========================================================================== //
//                                                                            //
//  Empty Status                                                              //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Compute empty status of currently addressed engine context.
sel #(.W(cfg_pkg::ENGS_N)) u_empty_sel (
  .i_x(empty_r), .i_sel(i_lk_engid_r), .o_y(engid_is_empty)
);

// ========================================================================== //
//                                                                            //
//  NEXT SRAM                                                                 //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//

// ========================================================================== //
//                                                                            //
//  PREV SRAM                                                                 //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//

// ========================================================================== //
//                                                                            //
//  Data SRAM                                                                 //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//

// ========================================================================== //
//                                                                            //
//  Microcode                                                                 //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign mem_uc_vld = 0;
assign mem_uc_engid = 0;
assign mem_uc_set_empty = 0;
assign mem_uc_clr_empty = 0;
assign mem_uc_head_vld = 0;
assign mem_uc_head_ptr = 0;
assign mem_uc_tail_vld = 0;
assign mem_uc_tail_vld = 0;

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Microcode (WRBK) output
assign o_mem_uc_vld_w = mem_uc_vld;
assign o_mem_uc_engid_w = mem_uc_engid;
assign o_mem_uc_set_empty_w = mem_uc_set_empty;
assign o_mem_uc_clr_empty_w = mem_uc_clr_empty;
assign o_mem_uc_head_vld_w = mem_uc_head_vld;
assign o_mem_uc_head_ptr_w = mem_uc_head_ptr;
assign o_mem_uc_tail_vld_w = mem_uc_tail_vld;
assign o_mem_uc_tail_ptr_w = mem_uc_tail_ptr;

endmodule : stk_pipe_lk

`include "unmacros.vh"
