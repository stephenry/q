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
, input wire logic                                i_lk_isfull_r
, input wire logic                                i_lk_dat_vld_r
, input wire logic [127:0]                        i_lk_dat_r

// -------------------------------------------------------------------------- //
// Allocation Interface
, input wire stk_pkg::ptr_t                       i_lk_ptr

// -------------------------------------------------------------------------- //
// MEM microcode update (reg).
, input wire logic                                i_wrbk_uc_vld_r
, input wire stk_pkg::engid_t                     i_wrbk_uc_engid_r
, input wire logic                                i_wrbk_uc_head_vld_r
, input wire stk_pkg::ptr_t                       i_wrbk_uc_head_ptr_r
, input wire logic                                i_wrbk_uc_tail_vld_r
, input wire stk_pkg::ptr_t                       i_wrbk_uc_tail_ptr_r

// -------------------------------------------------------------------------- //
// Tail SRAM interfaces
, output wire logic [stk_pkg::BANKS_N - 1:0]      o_lk_prev_ptr_ce_w
, output wire logic [stk_pkg::BANKS_N - 1:0]      o_lk_prev_ptr_oe_w
, output wire stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]
                                                  o_lk_prev_ptr_addr_w
, output wire stk_pkg::ptr_t [stk_pkg::BANKS_N - 1:0]
                                                  o_lk_prev_ptr_din_w

// -------------------------------------------------------------------------- //
// Data SRAM interfaces
, output wire logic [stk_pkg::BANKS_N - 1:0]      o_lk_dat_ce_w
, output wire logic [stk_pkg::BANKS_N - 1:0]      o_lk_dat_oe_w
, output wire stk_pkg::line_id_t                  o_lk_dat_addr_w
, output wire logic [127:0]                       o_lk_dat_din_w

// -------------------------------------------------------------------------- //
// MEM microcode update (nxt).
, output wire logic                               o_mem_uc_vld_w
, output wire stk_pkg::engid_t                    o_mem_uc_engid_w
, output wire [stk_pkg::BANKS_N - 1:0]            o_mem_uc_bankid_w
, output wire stk_pkg::opcode_t                   o_mem_uc_opcode_w
, output wire stk_pkg::status_t                   o_mem_uc_status_w
, output wire logic                               o_mem_uc_head_vld_w
, output wire logic                               o_mem_uc_head_dord_w
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
logic                                   cmd_is_push;
logic                                   cmd_is_pop;
logic                                   cmd_is_inv;

//
logic                                   mem_uc_vld;
stk_pkg::engid_t                        mem_uc_engid;
logic [stk_pkg::BANKS_N - 1:0]          mem_uc_bankid;
stk_pkg::opcode_t                       mem_uc_opcode;
stk_pkg::status_t                       mem_uc_status;
logic                                   mem_uc_head_vld;
logic                                   mem_uc_head_dord;
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

// Pointer Decoders
//
logic [stk_pkg::BANKS_N - 1:0]          lk_ptr_bank_id_d;
logic [stk_pkg::BANKS_N - 1:0]          head_ptr_bank_id_d;

// Command Status
//
logic                                   status_pop_from_empty;
logic                                   status_push_to_full;
logic                                   status_okay;

// Empty Status
//
logic [cfg_pkg::ENGS_N - 1:0]           lk_engid_d;
logic                                   engid_is_empty;
logic                                   empty_set_engid;
logic [cfg_pkg::ENGS_N - 1:0]           empty_set;
logic                                   empty_clr_engid;
logic [cfg_pkg::ENGS_N - 1:0]           empty_clr;
`Q_DFFR(logic [cfg_pkg::ENGS_N - 1:0], empty, '1, clk);

// Tail SRAM
logic [stk_pkg::BANKS_N - 1:0]          lk_prev_ptr_ce_w;
logic [stk_pkg::BANKS_N - 1:0]          lk_prev_ptr_oe_w;
stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]
                                        lk_prev_ptr_addr_w;
stk_pkg::ptr_t [stk_pkg::BANKS_N - 1:0]
                                        lk_prev_ptr_din_w;

// Data SRAM
logic [stk_pkg::BANKS_N - 1:0]          lk_dat_ce_w;
logic [stk_pkg::BANKS_N - 1:0]          lk_dat_oe_w;
stk_pkg::line_id_t                      lk_dat_addr_w;
logic [127:0]                           lk_dat_din_w;

// ========================================================================== //
//                                                                            //
//  Decoder                                                                   //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Command decoder
assign cmd_is_push = i_lk_vld_r & (i_lk_opcode_r == stk_pkg::OPCODE_PUSH);
assign cmd_is_pop = i_lk_vld_r & (i_lk_opcode_r == stk_pkg::OPCODE_POP);
assign cmd_is_inv = i_lk_vld_r & (i_lk_opcode_r == stk_pkg::OPCODE_INV);

// ========================================================================== //
//                                                                            //
//  Head Pointer Table                                                        //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Read Interface
assign rf_head_ra = i_lk_engid_r;

// -------------------------------------------------------------------------- //
// Write Interface:
assign rf_head_wen = i_wrbk_uc_vld_r & i_wrbk_uc_head_vld_r;
assign rf_head_wa = i_wrbk_uc_engid_r;
assign rf_head_wdata = i_wrbk_uc_head_ptr_r;

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
// Read Interface
assign rf_tail_ra = i_lk_engid_r;

// -------------------------------------------------------------------------- //
// Write Interface
assign rf_tail_wen = i_wrbk_uc_vld_r & i_wrbk_uc_tail_vld_r;
assign rf_tail_wa = i_wrbk_uc_engid_r;
assign rf_tail_wdata = i_wrbk_uc_tail_ptr_r;

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
//  Pointer Decoders                                                          //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
dec #(.W(stk_pkg::BANKS_N)) u_lk_ptr_bank_id_dec (
  .i_x(i_lk_ptr.bnk_id), .o_y(lk_ptr_bank_id_d)
);

// -------------------------------------------------------------------------- //
//
dec #(.W(stk_pkg::BANKS_N)) u_head_ptr_bank_id_dec (
  .i_x(rf_head_rdata.bnk_id), .o_y(head_ptr_bank_id_d)
);

// ========================================================================== //
//                                                                            //
//  Command Status                                                            //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
// Error: Attempt to pop from empty stack.
assign status_pop_from_empty = (cmd_is_pop | cmd_is_inv) & engid_is_empty;

// Error: Attempt to push to full stack (no free slots exist).
assign status_push_to_full = (cmd_is_push & i_lk_isfull_r);

// Success: No error conditions!
assign status_okay = ~(status_pop_from_empty | status_push_to_full);

// -------------------------------------------------------------------------- //
//
assign mem_uc_status =
    ({stk_pkg::STATUS_W{status_okay}}           & stk_pkg::STATUS_OKAY)
  | ({stk_pkg::STATUS_W{status_pop_from_empty}} & stk_pkg::STATUS_ERREMPTY)
  | ({stk_pkg::STATUS_W{status_push_to_full}}   & stk_pkg::STATUS_ERRFULL);

// ========================================================================== //
//                                                                            //
//  Empty Status                                                              //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Decode current engine-id.
//
dec #(.W(cfg_pkg::ENGS_N)) u_dec_qpush_engid (
  .i_x(i_lk_engid_r), .o_y(lk_engid_d)
);

// -------------------------------------------------------------------------- //
// Compute empty status of currently addressed engine context.
//
sel #(.W(cfg_pkg::ENGS_N)) u_empty_sel (
  .i_x(empty_r), .i_sel(i_lk_engid_r), .o_y(engid_is_empty)
);

// -------------------------------------------------------------------------- //
// Compute update to EMPTY set.
//
assign empty_clr_engid = (cmd_is_push & engid_is_empty);

assign empty_clr = lk_engid_d & {cfg_pkg::ENGS_N{empty_clr_engid}};

assign empty_set_engid =
  (cmd_is_pop & (rf_head_rdata == rf_tail_rdata) & (~engid_is_empty));

assign empty_set = lk_engid_d & {cfg_pkg::ENGS_N{empty_set_engid}};

// Set/Clear update.
assign empty_w = (~empty_clr) & (empty_r | empty_set);

// ========================================================================== //
//                                                                            //
//  PREV SRAM                                                                 //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
// Operations to PREV table for each command:
//
//  (1) PUSH - Write old head to newly allocated slot.
//
//  (2) POP  - Read current head to retrieve new (prior) head.
//
//  (3) INV  - As in (2), except access to DAT ram is killed.
//
for (genvar bank = 0; bank < stk_pkg::BANKS_N; bank++) begin : prev_bank_GEN

//
assign lk_prev_ptr_ce_w [bank] =
    ( cmd_is_push                  & lk_ptr_bank_id_d [bank])    // (1)
  | ((cmd_is_pop | cmd_is_inv)     & head_ptr_bank_id_d [bank]); // (2, 3)

//
assign lk_prev_ptr_oe_w [bank] =
    ((cmd_is_pop | cmd_is_inv)     & head_ptr_bank_id_d [bank]); // (2, 3)

//
assign lk_prev_ptr_addr_w [bank] =
    ({stk_pkg::LINE_ID_W{ cmd_is_push}} & i_lk_ptr.line_id)      // (1)
  | ({stk_pkg::LINE_ID_W{~cmd_is_push}} & rf_head_rdata.line_id);// (2, 3)

//
assign lk_prev_ptr_din_w [bank] = rf_head_rdata;

end : prev_bank_GEN

// ========================================================================== //
//                                                                            //
//  Data SRAM                                                                 //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
// Operations to DATA table for each command:
//
//  (1) PUSH - Write new descriptor to HEAD pointer.
//
//  (2) POP  - Read descriptor from current HEAD pointer.
//
//  (3) INV  - NOP

for (genvar bank = 0; bank < stk_pkg::BANKS_N; bank++) begin : data_bank_GEN

//
assign lk_dat_ce_w [bank] =
    (cmd_is_push                   & lk_ptr_bank_id_d [bank])    // (1)
  | (cmd_is_pop                    & head_ptr_bank_id_d [bank]); // (2)

//
assign lk_dat_oe_w [bank] =
    (cmd_is_pop                    & head_ptr_bank_id_d [bank]); // (2)

end : data_bank_GEN

//
assign lk_dat_addr_w =
    ({stk_pkg::LINE_ID_W{cmd_is_push}} & i_lk_ptr.line_id)       // (1)
  | ({stk_pkg::LINE_ID_W{ cmd_is_pop}} & rf_head_rdata.line_id); // (2)

//
assign lk_dat_din_w  = i_lk_dat_r;

// ========================================================================== //
//                                                                            //
//  Microcode                                                                 //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign mem_uc_vld = i_lk_vld_r;
assign mem_uc_engid = i_lk_engid_r;
assign mem_uc_bankid = cmd_is_push ? lk_ptr_bank_id_d : head_ptr_bank_id_d;
assign mem_uc_opcode = i_lk_opcode_r;

// -------------------------------------------------------------------------- //
//
assign mem_uc_head_vld = (cmd_is_push | cmd_is_pop | cmd_is_inv);
assign mem_uc_head_dord = (cmd_is_pop | cmd_is_inv);
assign mem_uc_head_ptr = i_lk_ptr;

// -------------------------------------------------------------------------- //
// The TAIL pointer remains largely invariant over time with the exception
// of the case where a PUSH is made to the empty stack. In this case, both
// head and tail pointers are updated simultaneously.
//
assign mem_uc_tail_vld = (cmd_is_push & engid_is_empty);
assign mem_uc_tail_ptr = i_lk_ptr;

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Microcode (WRBK) output
assign o_mem_uc_vld_w = mem_uc_vld;
assign o_mem_uc_engid_w = mem_uc_engid;
assign o_mem_uc_bankid_w = mem_uc_bankid;
assign o_mem_uc_opcode_w = mem_uc_opcode;
assign o_mem_uc_status_w = mem_uc_status;
assign o_mem_uc_head_vld_w = mem_uc_head_vld;
assign o_mem_uc_head_dord_w = mem_uc_head_dord;
assign o_mem_uc_head_ptr_w = mem_uc_head_ptr;
assign o_mem_uc_tail_vld_w = mem_uc_tail_vld;
assign o_mem_uc_tail_ptr_w = mem_uc_tail_ptr;

assign o_lk_prev_ptr_ce_w = lk_prev_ptr_ce_w;
assign o_lk_prev_ptr_oe_w = lk_prev_ptr_oe_w;
assign o_lk_prev_ptr_addr_w = lk_prev_ptr_addr_w;
assign o_lk_prev_ptr_din_w = lk_prev_ptr_din_w;

assign o_lk_dat_ce_w = lk_dat_ce_w;
assign o_lk_dat_oe_w = lk_dat_oe_w;
assign o_lk_dat_addr_w = lk_dat_addr_w;
assign o_lk_dat_din_w = lk_dat_din_w;

endmodule : stk_pipe_lk

`include "unmacros.vh"
