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
// Tail SRAM interfaces
  input wire logic [stk_pkg::BANKS_N - 1:0]       i_lk_prev_ptr_ce_r
, input wire logic [stk_pkg::BANKS_N - 1:0]       i_lk_prev_ptr_oe_r
, input wire stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]
                                                  i_lk_prev_ptr_addr_r
, input wire stk_pkg::ptr_t [stk_pkg::BANKS_N - 1:0]
                                                  i_lk_prev_ptr_din_r

// -------------------------------------------------------------------------- //
// Data SRAM interfaces
, input wire logic [stk_pkg::BANKS_N - 1:0]       i_lk_dat_ce_r
, input wire logic [stk_pkg::BANKS_N - 1:0]       i_lk_dat_oe_r
, input wire stk_pkg::line_id_t                   i_lk_dat_addr_r
, input wire [127:0]                              i_lk_dat_din_r

// -------------------------------------------------------------------------- //
// Lookup ("LK") microcode
, input wire logic                                i_mem_uc_vld_r
, input wire stk_pkg::engid_t                     i_mem_uc_engid_r
, input wire logic [stk_pkg::BANKS_N - 1:0]       i_mem_uc_bankid_r
, input wire stk_pkg::opcode_t                    i_mem_uc_opcode_r
, input wire stk_pkg::status_t                    i_mem_uc_status_r
, input wire logic                                i_mem_uc_islast_r
, input wire logic                                i_mem_uc_head_vld_r
, input wire logic                                i_mem_uc_head_dord_r
, input wire stk_pkg::ptr_t                       i_mem_uc_head_ptr_r
, input wire logic                                i_mem_uc_tail_vld_r
, input wire stk_pkg::ptr_t                       i_mem_uc_tail_ptr_r

// -------------------------------------------------------------------------- //
// Writeback ("WRBK") microcode
, output wire logic                               o_wrbk_uc_vld_w
, output wire stk_pkg::engid_t                    o_wrbk_uc_engid_w
, output wire stk_pkg::status_t                   o_wrbk_uc_status_w
, output wire stk_pkg::opcode_t                   o_wrbk_uc_opcode_w
, output wire logic                               o_wrbk_uc_islast_w
, output wire logic                               o_wrbk_uc_head_vld_w
, output wire stk_pkg::ptr_t                      o_wrbk_uc_head_ptr_w
, output wire logic                               o_wrbk_uc_tail_vld_w
, output wire stk_pkg::ptr_t                      o_wrbk_uc_tail_ptr_w
, output wire logic                               o_wrbk_uc_dat_vld
, output wire logic [127:0]                       o_wrbk_uc_dat_w

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

// PREV Pointer SRAM
//
stk_pkg::ptr_t [stk_pkg::BANKS_N - 1:0] lk_prev_ptr_dout;
stk_pkg::ptr_t                          lk_prev_ptr;

logic [stk_pkg::BANKS_N - 1:0][127:0]   lk_dat_dout;
logic [127:0]                           lk_dat;

`Q_DFFR(logic, mdat_uc_vld, 1'b0, clk);
`Q_DFFE(stk_pkg::engid_t, mdat_uc_engid, mdat_uc_vld_w, clk);
`Q_DFFE(logic [stk_pkg::BANKS_N - 1:0], mdat_uc_bankid, mdat_uc_vld_w, clk);
`Q_DFFE(stk_pkg::opcode_t, mdat_uc_opcode, mdat_uc_vld_w, clk);
`Q_DFFE(stk_pkg::status_t, mdat_uc_status, mdat_uc_vld_w, clk);
`Q_DFFE(logic, mdat_uc_islast, mdat_uc_vld_w, clk);
`Q_DFFE(logic, mdat_uc_head_vld, mdat_uc_vld_w, clk);
`Q_DFFE(logic, mdat_uc_head_dord, mdat_uc_vld_w, clk);
`Q_DFFE(stk_pkg::ptr_t, mdat_uc_head_ptr, mdat_uc_vld_w, clk);
`Q_DFFE(logic, mdat_uc_tail_vld, mdat_uc_vld_w, clk);
`Q_DFFE(stk_pkg::ptr_t, mdat_uc_tail_ptr, mdat_uc_vld_w, clk);

// Microcode
//
logic                                   mem_uc_vld;
stk_pkg::engid_t                        mem_uc_engid;
logic [stk_pkg::BANKS_N - 1:0]          mem_uc_bankid;
stk_pkg::opcode_t                       mem_uc_opcode;
stk_pkg::status_t                       mem_uc_status;
logic                                   mem_uc_islast;
logic                                   mem_uc_head_vld;
logic                                   mem_uc_head_dord;
stk_pkg::ptr_t                          mem_uc_head_ptr;
logic                                   mem_uc_tail_vld;
stk_pkg::ptr_t                          mem_uc_tail_ptr;

// Decoder (MDAT)
//
logic                                   mdat_cmd_is_push;
logic                                   mdat_cmd_is_pop;
logic                                   mdat_cmd_is_inv;

//
logic                                   wrbk_uc_vld;
stk_pkg::engid_t                        wrbk_uc_engid;
stk_pkg::status_t                       wrbk_uc_status;
stk_pkg::opcode_t                       wrbk_uc_opcode;
logic [stk_pkg::BANKS_N - 1:0]          wrbk_uc_bankid;
logic                                   wrbk_uc_islast;
logic                                   wrbk_uc_head_vld;
logic                                   wrbk_uc_head_dord;
stk_pkg::ptr_t                          wrbk_uc_head_ptr;
logic                                   wrbk_uc_tail_vld;
stk_pkg::ptr_t                          wrbk_uc_tail_ptr;
logic                                   wrbk_uc_dat_vld;
logic [127:0]                           wrbk_uc_dat;

// ========================================================================== //
//                                                                            //
//  Previous (PREV) Pointer SRAM                                              //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
for (genvar bnk = 0; bnk < stk_pkg::BANKS_N; bnk++) begin : prev_sram_GEN

stk_pipe_mem_prev_sram u_stk_pipe_mem_prev_sram (
//
  .i_addr                     (i_lk_prev_ptr_addr_r [bnk])
, .i_din                      (i_lk_prev_ptr_din_r [bnk])
, .i_ce                       (i_lk_prev_ptr_ce_r [bnk])
, .i_oe                       (i_lk_prev_ptr_oe_r [bnk])
//
, .o_dout                     (lk_prev_ptr_dout [bnk])
//
, .clk                        (clk)
);

end : prev_sram_GEN

// -------------------------------------------------------------------------- //
// Data SRAM demux.
mux #(.N(stk_pkg::BANKS_N), .W(stk_pkg::PTR_W)) u_prev_mux (
  .i_x(lk_prev_ptr_dout), .i_sel(mdat_uc_bankid_r), .o_y(lk_prev_ptr)
);

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
  .i_addr                     (i_lk_dat_addr_r)
, .i_din                      (i_lk_dat_din_r)
, .i_ce                       (i_lk_dat_ce_r [bnk])
, .i_oe                       (i_lk_dat_oe_r [bnk])
//
, .o_dout                     (lk_dat_dout [bnk])
//
, .clk                        (clk)
);

end : data_sram_GEN

// -------------------------------------------------------------------------- //
// Data SRAM demux.
mux #(.N(stk_pkg::BANKS_N), .W(128)) u_data_mux (
  .i_x(lk_dat_dout), .i_sel(mdat_uc_bankid_r), .o_y(lk_dat)
);

// ========================================================================== //
//                                                                            //
//  Microcode (MEM)                                                           //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// MEM -> MDAT
assign mdat_uc_vld_w = i_mem_uc_vld_r;
assign mdat_uc_engid_w = i_mem_uc_engid_r;
assign mdat_uc_bankid_w = i_mem_uc_bankid_r;
assign mdat_uc_opcode_w = i_mem_uc_opcode_r;
assign mdat_uc_status_w = i_mem_uc_status_r;
assign mdat_uc_islast_w = i_mem_uc_islast_r;
assign mdat_uc_head_vld_w = i_mem_uc_head_vld_r;
assign mdat_uc_head_dord_w = i_mem_uc_head_dord_r;
assign mdat_uc_head_ptr_w = i_mem_uc_head_ptr_r;
assign mdat_uc_tail_vld_w = i_mem_uc_tail_vld_r;
assign mdat_uc_tail_ptr_w = i_mem_uc_tail_ptr_r;

// ========================================================================== //
//                                                                            //
//  Decoder (MDAT)                                                            //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Command decoder
assign mdat_cmd_is_push = (mdat_uc_opcode_r == stk_pkg::OPCODE_PUSH);
assign mdat_cmd_is_pop = (mdat_uc_opcode_r == stk_pkg::OPCODE_POP);
assign mdat_cmd_is_inv = (mdat_uc_opcode_r == stk_pkg::OPCODE_INV);

// ========================================================================== //
//                                                                            //
//  Microcode (MDAT)                                                          //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// MDAT -> WRBK
assign wrbk_uc_vld = mdat_uc_vld_r;
assign wrbk_uc_engid = mdat_uc_engid_r;
assign wrbk_uc_status = mdat_uc_status_r;
assign wrbk_uc_opcode = mdat_uc_opcode_r;
assign wrbk_uc_islast = mdat_uc_islast_r;

assign wrbk_uc_head_vld = mdat_uc_head_vld_r;
assign wrbk_uc_head_ptr =
  mdat_uc_head_dord_r ? lk_prev_ptr : mdat_uc_head_ptr_r;

assign wrbk_uc_tail_vld = mdat_uc_tail_vld_r;
assign wrbk_uc_tail_ptr = mdat_uc_tail_ptr_r;

assign wrbk_uc_dat_vld = mdat_cmd_is_pop;
assign wrbk_uc_dat = lk_dat;

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

assign o_wrbk_uc_vld_w = wrbk_uc_vld;
assign o_wrbk_uc_engid_w = wrbk_uc_engid;
assign o_wrbk_uc_status_w = wrbk_uc_status;
assign o_wrbk_uc_opcode_w = wrbk_uc_opcode;
assign o_wrbk_uc_islast_w = wrbk_uc_islast;
assign o_wrbk_uc_head_vld_w = wrbk_uc_head_vld;
assign o_wrbk_uc_head_ptr_w = wrbk_uc_head_ptr;
assign o_wrbk_uc_tail_vld_w = wrbk_uc_tail_vld;
assign o_wrbk_uc_tail_ptr_w = wrbk_uc_tail_ptr;
assign o_wrbk_uc_dat_vld = wrbk_uc_dat_vld;
assign o_wrbk_uc_dat_w = wrbk_uc_dat;

endmodule : stk_pipe_mem

`include "unmacros.vh"
