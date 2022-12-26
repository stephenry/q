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
`include "stk/stk_pkg.vh"
`include "macros.vh"

module stk_pipe_al (

// -------------------------------------------------------------------------- //
// Admission ("ad") Stage Interface
  input wire logic                                i_ad_alloc
, output wire logic                               o_ad_empty_r
, output wire logic                               o_ad_busy_r

// -------------------------------------------------------------------------- //
// Lookup ("lk") Stage Interface
, output wire stk_pkg::ptr_t                      o_lk_ptr_w

// -------------------------------------------------------------------------- //
// Descriptor Return Interface
, input wire logic                                i_dealloc_vld
, input wire stk_pkg::ptr_t                       i_dealloc_ptr

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

// Initialization Logic
//
`Q_DFFR(logic, reset_init, 1'b1, clk);
logic                                        init;
logic                                        init_wen_r;
stk_pkg::line_id_t                           init_waddr_r;
stk_pkg::line_id_t                           init_wdata_r;
logic                                        init_busy_r;

// -------------------------------------------------------------------------- //
// Bank Selection Logic
//
logic                                        stk_collision;
logic [stk_pkg::BANKS_N - 1:0]               stk_bnk_popsel_req_d;
logic                                        stk_bnk_popsel_ack;
logic [stk_pkg::BANKS_N - 1:0]               stk_bnk_popsel_gnt_d;

// -------------------------------------------------------------------------- //
// Free Pool Stack Control
//
logic [stk_pkg::BANKS_N - 1:0]               dealloc_ptr_bnk_id_d;
logic [stk_pkg::BANKS_N - 1:0]               stk_bnk_push;
logic [stk_pkg::BANKS_N - 1:0]               stk_bnk_pop;
logic [stk_pkg::BANKS_N - 1:0]               stk_bnk_mem_wen;
logic [stk_pkg::BANKS_N - 1:0]               stk_bnk_mem_ren;
stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]  stk_bnk_mem_addr;
`Q_DFF(logic [stk_pkg::BANKS_N - 1:0], stk_bnk_empty, clk);
`Q_DFF(logic, ad_empty, clk);

// Memory
//
stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]  mem_addr;
stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]  mem_din;
logic [stk_pkg::BANKS_N - 1:0]               mem_ce;
logic [stk_pkg::BANKS_N - 1:0]               mem_oe;
stk_pkg::line_id_t [stk_pkg::BANKS_N - 1:0]  mem_dout;

// -------------------------------------------------------------------------- //
// Admission Stage Microcode
//
`Q_DFF(logic, lk_bypass, clk);
logic                                        lk_bypass_ptr_en;
`Q_DFFE(stk_pkg::ptr_t, lk_bypass_ptr, lk_bypass_ptr_en, clk);
logic                                        lk_en;
`Q_DFFE(stk_pkg::bank_id_t, lk_bank_id, lk_en, clk);

// -------------------------------------------------------------------------- //
// Lookup Stage Microcode
//
stk_pkg::ptr_t                               lk_ptr_w;
stk_pkg::line_id_t                           lk_mem_line_w;
stk_pkg::ptr_t                               lk_mem_w;

// ========================================================================== //
//                                                                            //
//  Initialization                                                            //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Stack Initialization Logic
assign init = reset_init_r;

stk_pipe_al_init u_stk_pipe_al_init (
//
  .o_init_wen_r               (init_wen_r)
, .o_init_waddr_r             (init_waddr_r)
, .o_init_wdata_r             (init_wdata_r)
//
, .i_init                     (init)
, .o_busy_r                   (init_busy_r)
//
, .clk                        (clk)
);

// ========================================================================== //
//                                                                            //
//  Bank Selection Logic                                                      //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// An allocation request occurs in the same cycle as a deallocation. In this
// case, immediately the deallocated descriptor to the request to avoid having
// to touch the allocators.
//
assign stk_collision = (i_ad_alloc & i_dealloc_vld);

// -------------------------------------------------------------------------- //
// Compute bank request set as those which have available descriptors;
// we do not collision conditions here specifically as we always reuse
// deallocated descriptors immediately.
//
assign stk_bnk_popsel_req_d = (~stk_bnk_empty_r);

// -------------------------------------------------------------------------- //
// Load balance across banks whenever a descriptor is allocated.
//
assign stk_bnk_popsel_ack = i_ad_alloc & (~i_dealloc_vld);

// -------------------------------------------------------------------------- //
// Compute nominated bank for non-bypassed case.
//
rr #(.W(stk_pkg::BANKS_N)) u_rr (
//
  .i_req                      (stk_bnk_popsel_req_d)
, .i_ack                      (stk_bnk_popsel_ack)
, .o_gnt                      (stk_bnk_popsel_gnt_d)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

// ========================================================================== //
//                                                                            //
//  Free Pool Stack control                                                   //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Decode line field of deallocated descriptor to identify home bank.
//
dec #(.W(stk_pkg::BANKS_N)) u_dec_qpush_engid (
  .i_x(i_dealloc_ptr.bnk_id), .o_y(dealloc_ptr_bnk_id_d)
);

for (genvar bnk = 0; bnk < stk_pkg::BANKS_N; bnk++) begin : stack_logical_GEN

assign stk_bnk_push [bnk] =
    init_wen_r                                                        // (1)
  | (i_dealloc_vld & dealloc_ptr_bnk_id_d [bnk] & ~stk_collision);    // (2)

assign stk_bnk_pop [bnk] =
  (i_ad_alloc & stk_bnk_popsel_gnt_d [bnk] & ~stk_collision);

end : stack_logical_GEN

// -------------------------------------------------------------------------- //
//
for (genvar bnk = 0; bnk < stk_pkg::BANKS_N; bnk++) begin : stack_cntrl_GEN

stack_cntrl #(.N(stk_pkg::C_BANK_LINES_N)) u_stack_cntrl (
//
  .i_push                     (stk_bnk_push [bnk])
, .i_pop                      (stk_bnk_pop [bnk])
//
, .o_mem_wen                  (stk_bnk_mem_wen [bnk])
, .o_mem_ren                  (stk_bnk_mem_ren [bnk])
, .o_mem_addr                 (stk_bnk_mem_addr [bnk])
//
, .o_full_w                   ()
, .o_empty_w                  (stk_bnk_empty_w [bnk])
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

end : stack_cntrl_GEN

// -------------------------------------------------------------------------- //
// Allocation is empty when all banks report empty status.
assign ad_empty_w = (stk_bnk_empty_w != '1);

// ========================================================================== //
//                                                                            //
//  Free Pool SRAM control                                                    //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
for (genvar bnk = 0; bnk < stk_pkg::BANKS_N; bnk++) begin : mem_wires_GEN

assign mem_addr [bnk] =
  init_wen_r ? init_waddr_r : stk_bnk_mem_addr [bnk];

assign mem_din [bnk] =
  init_wen_r ? init_wdata_r : i_dealloc_ptr.line_id;

assign mem_ce [bnk] = (stk_bnk_mem_wen [bnk] | stk_bnk_mem_ren [bnk]);

assign mem_oe [bnk] = stk_bnk_mem_ren [bnk];

end : mem_wires_GEN

// -------------------------------------------------------------------------- //
//
for (genvar bnk = 0; bnk < stk_pkg::BANKS_N; bnk++) begin : stack_mem_GEN

stk_pipe_al_ptr_sram u_stk_pipe_al_ptr_sram (
//
  .i_addr                     (mem_addr [bnk])
, .i_din                      (mem_din [bnk])
, .i_ce                       (mem_ce [bnk])
, .i_oe                       (mem_oe [bnk])
, .o_dout                     (mem_dout [bnk])
//
, .clk                        (clk)
);

end : stack_mem_GEN

// ========================================================================== //
//                                                                            //
//  Admission Stage (AD)                                                      //
//                                                                            //
// ========================================================================== //

// Enable bus on allocation request.
assign lk_en = i_ad_alloc;

// Flag indicating that descriptor is to be sourced from the bypass
// and not the RAM.
assign lk_bypass_w = stk_collision;

// Descriptor bypass bus and enable.
assign lk_bypass_ptr_en = (lk_en & lk_bypass_w);
assign lk_bypass_ptr_w = i_dealloc_ptr;

// For non-bypassed case, encode bank select for subsequent downstream
// muxing.
enc #(.W(stk_pkg::BANKS_N)) u_rr_enc (
  .i_x(stk_bnk_popsel_gnt_d), .o_y(lk_bank_id_w)
);

// ========================================================================== //
//                                                                            //
//  Lookup Stage (LK)                                                         //
//                                                                            //
// ========================================================================== //

// Select bank output.
//
muxe #(.N(stk_pkg::BANKS_N), .W(stk_pkg::BANK_LINE_OFFSET_W)) u_lk_mem_muxe (
  .i_x (mem_dout), .i_sel (lk_bank_id_r), .o_y (lk_mem_line_w)
);

// Compose final pointer returning from memory.
//
assign lk_mem_w = '{bnk_id:lk_bank_id_r, line_id:lk_mem_line_w};

// Select between bypassed or non-bypassed buses.
//
assign lk_ptr_w =
    ({stk_pkg::PTR_W{ lk_bypass_r}} & lk_bypass_ptr_r)
  | ({stk_pkg::PTR_W{~lk_bypass_r}} & lk_mem_w);

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

assign o_lk_ptr_w = lk_ptr_w;
assign o_ad_busy_r = init_busy_r;
assign o_ad_empty_r = ad_empty_r;

endmodule : stk_pipe_al

`include "unmacros.vh"
