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

module stk_pipe_ad (
// -------------------------------------------------------------------------- //
// Command Interface:
  input wire stk_pkg::opcode_t [cfg_pkg::ENGS_N - 1:0]
                                                  i_cmd_opcode
, input wire logic [cfg_pkg::ENGS_N - 1:0][127:0] i_cmd_dat
//
, output wire logic [cfg_pkg::ENGS_N - 1:0]       o_cmd_ack

// -------------------------------------------------------------------------- //
// LK ("Look-up") Stage Microcode:
, output wire logic                               o_lk_vld_w
, output wire stk_pkg::engid_t                    o_lk_engid_w
, output wire stk_pkg::opcode_t                   o_lk_opcode_w
, output wire logic                               o_lk_dat_vld_w
, output wire logic [127:0]                       o_lk_dat_w

// -------------------------------------------------------------------------- //
// Allocation Interface:
, input wire logic                                i_al_empty_r
, input wire logic                                i_al_busy
//
, output wire logic                               o_al_alloc

// -------------------------------------------------------------------------- //
// Response Interface:
, input wire logic [cfg_pkg::ENGS_N - 1:0]        i_rsp_vld

// -------------------------------------------------------------------------- //
// Clk/Reset
, input wire logic                                clk
, input wire logic                                arst_n
);

typedef struct packed {
  logic [$clog2(cfg_pkg::ENGS_N) - 1:0]    id;
  logic [127:0]                            dat;
} qpush_t;
localparam int QPUSH_W = $bits(qpush_t);

// Command Logic:
//
logic [cfg_pkg::ENGS_N - 1:0]           cmd_vld;
logic [cfg_pkg::ENGS_N - 1:0]           cmd_is_push;
logic [cfg_pkg::ENGS_N - 1:0]           cmd_is_pop;
logic [cfg_pkg::ENGS_N - 1:0]           cmd_is_inv;
logic [cfg_pkg::ENGS_N - 1:0]           cmd_ack;

// Enqueue logic:
//
logic [cfg_pkg::ENGS_N - 1:0]           enq_req_d;
logic [cfg_pkg::ENGS_N - 1:0]           enq_gnt_d;
stk_pkg::engid_t                        enq_gnt;
logic                                   enq_ack;

// "Push" Command Queue:
//
logic                                   qpush_push;
logic [127:0]                           qpush_push_dat_dat;
logic                                   qpush_pop;
qpush_t                                 qpush_push_dat;
qpush_t                                 qpush_pop_dat;
logic [cfg_pkg::ENGS_N - 1:0]           qpush_pop_dat_engid_d;
`Q_DFF(logic, qpush_full, clk);
`Q_DFF(logic, qpush_empty, clk);

// "Pop" Command Queue:
//
logic                                   qpop_push;
stk_pkg::engid_t                        qpop_push_dat;
logic                                   qpop_pop;
stk_pkg::engid_t                        qpop_pop_dat;
`Q_DFF(logic, qpop_full, clk);
`Q_DFF(logic, qpop_empty, clk);
logic [cfg_pkg::ENGS_N - 1:0]          qpop_pop_dat_engid_d;

// "Invalidation" Command Queue:
//
logic                                   qinv_push;
stk_pkg::engid_t                        qinv_push_dat;
`Q_DFF(logic, qinv_full, clk);
`Q_DFF(logic, qinv_empty, clk);

// "Active" Set:
//
logic [cfg_pkg::ENGS_N - 1:0]           active_set;
logic [cfg_pkg::ENGS_N - 1:0]           active_clr;
`Q_DFFR(logic [cfg_pkg::ENGS_N - 1:0], active, '0, clk);

// Dequeue logic:
//
logic [2:0]                             deq_req_d;
logic [2:0]                             deq_gnt_d;
logic                                   deq_ack;

// Output logic:
logic                                   lk_vld;
stk_pkg::engid_t                        lk_engid;
logic [cfg_pkg::ENGS_N - 1:0]           lk_engid_d;
stk_pkg::opcode_t                       lk_opcode;
logic                                   lk_dat_vld;
logic [127:0]                           lk_dat;

// -------------------------------------------------------------------------- //
// Command decoder:
for (genvar ch = 0; ch < cfg_pkg::ENGS_N; ch++) begin : cmd_decoder

assign cmd_vld [ch] = (i_cmd_opcode [ch] != stk_pkg::OPCODE_NOP);
assign cmd_is_push [ch] = (i_cmd_opcode [ch] == stk_pkg::OPCODE_PUSH);
assign cmd_is_pop [ch] = (i_cmd_opcode [ch] == stk_pkg::OPCODE_POP);
assign cmd_is_inv [ch] = (i_cmd_opcode [ch] == stk_pkg::OPCODE_INV);

end : cmd_decoder

// -------------------------------------------------------------------------- //

assign enq_req_d = cmd_vld &
  ((cmd_is_push & {cfg_pkg::ENGS_N{~qpush_full_r}}) |
   (cmd_is_pop  & {cfg_pkg::ENGS_N{~qpop_full_r}}));

rr #(.W(cfg_pkg::ENGS_N)) u_rr_enq (
//
  .i_req                      (enq_req_d)
, .i_ack                      (enq_ack)
, .o_gnt                      (enq_gnt_d)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

enc #(.W(cfg_pkg::ENGS_N)) u_rr_enq_gnt_enc (
  .i_x(enq_gnt_d), .o_y(enq_gnt)
);

assign enq_ack = (qpush_push | qpop_push | qinv_push);

assign cmd_ack = (enq_gnt_d & {cfg_pkg::ENGS_N{enq_ack}});

// -------------------------------------------------------------------------- //
//
assign qpush_push = (enq_gnt_d & cmd_is_push) != '0;

mux #(.N(cfg_pkg::ENGS_N), .W(128)) u_enq_mux (
  .i_x(i_cmd_dat), .i_sel(enq_gnt_d), .o_y(qpush_push_dat_dat)
);

assign qpush_push_dat = '{id:enq_gnt, dat:qpush_push_dat_dat};

queue_rf #(.N(2), .W(QPUSH_W)) u_qpush (
//
  .i_push                     (qpush_push)
, .i_push_dat                 (qpush_push_dat)
//
, .i_pop                      (qpush_pop)
, .o_pop_dat                  (qpush_pop_dat)
//
, .o_full_w                   (qpush_empty_w)
, .o_empty_w                  (qpush_full_w)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

dec #(.W(cfg_pkg::ENGS_N)) u_dec_qpush_engid (
  .i_x(qpush_pop_dat.id), .o_y(qpush_pop_dat_engid_d)
);

// -------------------------------------------------------------------------- //
//
assign qpop_push = (enq_gnt_d & cmd_is_pop) != '0;
assign qpop_push_dat = enq_gnt;

queue_rf #(.N(2), .W(stk_pkg::ENGID_W)) u_qpop (
//
  .i_push                     (qpop_push)
, .i_push_dat                 (qpop_push_dat)
//
, .i_pop                      (qpop_pop)
, .o_pop_dat                  (qpop_pop_dat)
//
, .o_full_w                   (qpop_empty_w)
, .o_empty_w                  (qpop_full_w)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

dec #(.W(cfg_pkg::ENGS_N)) u_dec_qpop_engid (
  .i_x(qpop_pop_dat), .o_y(qpop_pop_dat_engid_d)
);

// -------------------------------------------------------------------------- //
//
assign qinv_push = (enq_gnt_d & cmd_is_inv) != '0;
assign qinv_push_dat = enq_gnt;

queue_rf #(.N(2), .W(stk_pkg::ENGID_W)) u_qinv (
//
  .i_push                     (qinv_push)
, .i_push_dat                 (qinv_push_dat)
//
, .i_pop                      ()
, .o_pop_dat                  ()
//
, .o_full_w                   (qinv_empty_w)
, .o_empty_w                  (qinv_full_w)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

// -------------------------------------------------------------------------- //
//
stk_pipe_ad_inv u_stk_pipe_ad_inv (
//
  .o_req_vld                  ()
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

// -------------------------------------------------------------------------- //
//

dec #(.W(cfg_pkg::ENGS_N)) u_dec (.i_x(lk_engid), .o_y(lk_engid_d));

assign active_set = (lk_engid_d & {cfg_pkg::ENGS_N{lk_vld}});
assign active_clr = i_rsp_vld;
assign active_w = (~active_clr) & (active_r | active_set);

// -------------------------------------------------------------------------- //
//

assign deq_req_d [0] =
  (~i_al_empty_r) & (~qpush_empty_r) & ((active_r & qpush_pop_dat_engid_d) == '0);

assign deq_req_d [1] =
  (~qpop_empty_r) & ((active_r & qpop_pop_dat_engid_d) == '0);

assign deq_req_d [2] =
  1'b0; // TODO

assign deq_ack = (~i_al_busy) & (deq_req_d != '0);

// -------------------------------------------------------------------------- //
//
rr #(.W(3)) u_rr (
//
  .i_req                      (deq_req_d)
, .i_ack                      (deq_ack)
, .o_gnt                      (deq_gnt_d)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

assign qpush_pop = deq_ack & deq_gnt_d [0];
assign qpop_pop = deq_ack & deq_gnt_d [1];

// -------------------------------------------------------------------------- //
// Output

assign lk_vld = deq_ack;

assign lk_engid =
    ({stk_pkg::ENGID_W{deq_gnt_d[0]}} & qpush_pop_dat.id)
  | ({stk_pkg::ENGID_W{deq_gnt_d[1]}} & qpop_pop_dat);

assign lk_opcode =
    ({stk_pkg::OPCODE_W{deq_gnt_d[0]}} & stk_pkg::OPCODE_PUSH)
  | ({stk_pkg::OPCODE_W{deq_gnt_d[1]}} & stk_pkg::OPCODE_POP);

assign lk_dat_vld = lk_vld & (lk_opcode == stk_pkg::OPCODE_PUSH);

assign lk_dat = qpush_pop_dat.dat;

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

// Command response:
assign o_cmd_ack = cmd_ack;

// Downstream microcode.
assign o_lk_vld_w = lk_vld;
assign o_lk_engid_w = lk_engid;
assign o_lk_opcode_w = lk_opcode;
assign o_lk_dat_vld_w = lk_dat_vld;
assign o_lk_dat_w = lk_dat;

endmodule : stk_pipe_ad
