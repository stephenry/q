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
, output wire logic                               o_lk_isfull_w
, output wire logic                               o_lk_dat_vld_w
, output wire logic [127:0]                       o_lk_dat_w

// -------------------------------------------------------------------------- //
// Allocation Interface:
, input wire logic                                i_al_empty_r
, input wire logic                                i_al_full_r
, input wire logic                                i_al_busy_r
//
, output wire logic                               o_al_alloc

// -------------------------------------------------------------------------- //
// Writeback ("WRBK") microcode
, output wire logic                               i_wrbk_uc_vld_r
, output wire stk_pkg::engid_t                    i_wrbk_uc_engid_r

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
`Q_DFFR(logic, qpush_full, 1'b0, clk);
`Q_DFFR(logic, qpush_empty, 1'b1, clk);

// "Pop" Command Queue:
//
logic                                   qpop_push;
stk_pkg::engid_t                        qpop_push_dat;
logic                                   qpop_pop;
stk_pkg::engid_t                        qpop_pop_dat;
`Q_DFFR(logic, qpop_full, 1'b0, clk);
`Q_DFFR(logic, qpop_empty, 1'b1, clk);

// "Invalidation" Command Queue:
//
logic                                   qinv_push;
stk_pkg::engid_t                        qinv_push_dat;
`Q_DFFR(logic, qinv_full, 1'b0, clk);
`Q_DFFR(logic, qinv_empty, 1'b1, clk);

// "Active" Set:
//
logic [cfg_pkg::ENGS_N - 1:0]           wrbk_uc_engid_d;
logic [cfg_pkg::ENGS_N - 1:0]           active_set;
logic [cfg_pkg::ENGS_N - 1:0]           active_clr;
`Q_DFFR(logic [cfg_pkg::ENGS_N - 1:0], active, '0, clk);

// Dequeue logic:
//
localparam int IDX_PUSH = 0;
localparam int IDX_POP = 1;
localparam int IDX_INV = 2;
//
logic                                   qpush_id_active;
logic                                   qpop_id_active;
logic [2:0]                             deq_req_d;
logic [2:0]                             deq_gnt_d;
logic                                   deq_ack;

// Output logic:
logic [cfg_pkg::ENGS_N - 1:0]           lk_engid_d;

logic                                   al_alloc;

// ========================================================================== //
//                                                                            //
//  Command Decoder                                                           //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Command decoder:
for (genvar ch = 0; ch < cfg_pkg::ENGS_N; ch++) begin : cmd_decoder

assign cmd_vld [ch] = (i_cmd_opcode [ch] != stk_pkg::OPCODE_NOP);
assign cmd_is_push [ch] = (i_cmd_opcode [ch] == stk_pkg::OPCODE_PUSH);
assign cmd_is_pop [ch] = (i_cmd_opcode [ch] == stk_pkg::OPCODE_POP);
assign cmd_is_inv [ch] = (i_cmd_opcode [ch] == stk_pkg::OPCODE_INV);

end : cmd_decoder

// ========================================================================== //
//                                                                            //
//  Enqueue Logic                                                             //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign enq_req_d = cmd_vld &
  ((cmd_is_push & {cfg_pkg::ENGS_N{~qpush_full_r}}) |
   (cmd_is_pop  & {cfg_pkg::ENGS_N{~qpop_full_r}}));

// -------------------------------------------------------------------------- //
//
rr #(.W(cfg_pkg::ENGS_N)) u_rr_enq (
//
  .i_req                      (enq_req_d)
, .i_ack                      (enq_ack)
, .o_gnt                      (enq_gnt_d)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

// -------------------------------------------------------------------------- //
//
enc #(.W(cfg_pkg::ENGS_N)) u_rr_enq_gnt_enc (
  .i_x(enq_gnt_d), .o_y(enq_gnt)
);

// -------------------------------------------------------------------------- //
//
assign enq_ack = (qpush_push | qpop_push | qinv_push);

// -------------------------------------------------------------------------- //
//
assign cmd_ack = (enq_gnt_d & {cfg_pkg::ENGS_N{enq_ack}});


// ========================================================================== //
//                                                                            //
//  "Push" Opcode Queue                                                       //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign qpush_push = (enq_gnt_d & cmd_is_push) != '0;

// -------------------------------------------------------------------------- //
//
assign qpush_pop = deq_ack & deq_gnt_d [0];

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
, .o_full_w                   (qpush_full_w)
, .o_empty_w                  (qpush_empty_w)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

// ========================================================================== //
//                                                                            //
//  "Pop" Opcode Queue                                                        //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign qpop_push = (enq_gnt_d & cmd_is_pop) != '0;

assign qpop_pop = deq_ack & deq_gnt_d [1];

assign qpop_push_dat = enq_gnt;

queue_rf #(.N(2), .W(stk_pkg::ENGID_W)) u_qpop (
//
  .i_push                     (qpop_push)
, .i_push_dat                 (qpop_push_dat)
//
, .i_pop                      (qpop_pop)
, .o_pop_dat                  (qpop_pop_dat)
//
, .o_full_w                   (qpop_full_w)
, .o_empty_w                  (qpop_empty_w)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

// ========================================================================== //
//                                                                            //
//  "INV" Opcode Queue                                                        //
//                                                                            //
// ========================================================================== //

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
, .o_full_w                   (qinv_full_w)
, .o_empty_w                  (qinv_empty_w)
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

dec #(.W(cfg_pkg::ENGS_N)) u_dec (.i_x(o_lk_engid_w), .o_y(lk_engid_d));

// ========================================================================== //
//                                                                            //
//  Requester Arbitration                                                     //
//                                                                            //
// We determining whether a push/pop command may issue, we do not             //
// consider the occupancy of the allocation unit. It would appear             //
// logical that we would hold-off push operations in favor of pop             //
// where all slots have been allocated, but we specifically allow             //
// these operations to fail to avoid possible deadlock conditions. We         //
// rely upon the high-levels of control, the issuing agent to                 //
// appropriately enforcing a logically correct ordering of push and           //
// pop operations.                                                            //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
sel #(.W(cfg_pkg::ENGS_N)) u_qpush_active_sel (
  .i_x(active_r), .i_sel(qpush_pop_dat.id), .o_y(qpush_id_active)
);

assign deq_req_d [IDX_PUSH] = (~qpush_empty_r) & (~qpush_id_active);

// -------------------------------------------------------------------------- //
//
sel #(.W(cfg_pkg::ENGS_N)) u_qpop_active_sel (
  .i_x(active_r), .i_sel(qpop_pop_dat), .o_y(qpop_id_active)
);

assign deq_req_d [IDX_POP] = (~qpop_empty_r) & (~qpop_id_active);

// -------------------------------------------------------------------------- //
//
assign deq_req_d [IDX_INV] =
  1'b0; // TODO

// -------------------------------------------------------------------------- //
//
assign deq_ack = (~i_al_busy_r) & (deq_req_d != '0);

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

// ========================================================================== //
//                                                                            //
//  Line Allocation Request (to AL-stage)                                     //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Request new line if outgoing request is a PUSH.
//
assign al_alloc = o_lk_vld_w & (o_lk_opcode_w == stk_pkg::OPCODE_PUSH);

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// On issue, set a bit in the active set to hold-off (backpressure) further
// commands belonging to the same engine-id.
assign active_set = (lk_engid_d & {cfg_pkg::ENGS_N{o_lk_vld_w}});

// -------------------------------------------------------------------------- //
// As responses egress the pipe, the engine-ID is decoded to clear the
// "active" bitmap. Once the associated bit has been cleared, ENGID commands
// are no longer backpressured and are able to be issued.
//
dec #(.W(cfg_pkg::ENGS_N)) u_wrbk_uc_engid_dec (
  .i_x(i_wrbk_uc_engid_r), .o_y(wrbk_uc_engid_d)
);

assign active_clr = {cfg_pkg::ENGS_N{i_wrbk_uc_vld_r}} & wrbk_uc_engid_d;

// -------------------------------------------------------------------------- //
// 'active' set derives as a standard set/clear update.
assign active_w = (~active_clr) & (active_r | active_set);

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Command response:
assign o_cmd_ack = cmd_ack;

// -------------------------------------------------------------------------- //
//
assign o_lk_vld_w = deq_ack;

// -------------------------------------------------------------------------- //
//
assign o_lk_engid_w =
    ({stk_pkg::ENGID_W{deq_gnt_d[IDX_PUSH]}} & qpush_pop_dat.id)
  | ({stk_pkg::ENGID_W{deq_gnt_d[IDX_POP]}} & qpop_pop_dat);

// -------------------------------------------------------------------------- //
//
assign o_lk_opcode_w =
    ({stk_pkg::OPCODE_W{deq_gnt_d[IDX_PUSH]}} & stk_pkg::OPCODE_PUSH)
  | ({stk_pkg::OPCODE_W{deq_gnt_d[IDX_POP]}} & stk_pkg::OPCODE_POP);

// -------------------------------------------------------------------------- //
//
assign o_lk_isfull_w = (deq_gnt_d[IDX_PUSH] & i_al_full_r);

// -------------------------------------------------------------------------- //
//
assign o_lk_dat_vld_w = o_lk_vld_w & (o_lk_opcode_w == stk_pkg::OPCODE_PUSH);

// -------------------------------------------------------------------------- //
//
assign o_lk_dat_w = qpush_pop_dat.dat;

// -------------------------------------------------------------------------- //
//
assign o_al_alloc = al_alloc;

endmodule : stk_pipe_ad
