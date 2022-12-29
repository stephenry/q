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

module stk_pipe_ad_inv (
// -------------------------------------------------------------------------- //
// Enqueue Interface
  input wire logic                                i_push
, input wire stk_pkg::engid_t                     i_push_dat

, output wire logic                               o_full_r

// -------------------------------------------------------------------------- //
// Issue Interface
, input wire logic                                i_iss_ack
//
, output wire logic                               o_iss_req
, output wire stk_pkg::engid_t                    o_iss_engid

// -------------------------------------------------------------------------- //
// Writeback Interface
, input wire logic                                i_wrbk_uc_vld_r
, input wire stk_pkg::engid_t                     i_wrbk_uc_engid_r
, input wire logic                                i_wrbk_uc_islast_r

// -------------------------------------------------------------------------- //
// Active Interface
, output wire logic [cfg_pkg::ENGS_N - 1:0]       o_active_set_d
, output wire logic [cfg_pkg::ENGS_N - 1:0]       o_active_clr_d

// -------------------------------------------------------------------------- //
// Response Interface
, output wire logic                               o_rsp_inv_kill

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

localparam int STATE_W = 3;

typedef enum logic [STATE_W - 1:0] {
  FSM_STATE_IDLE = STATE_W'('b001)
, FSM_STATE_INIT = STATE_W'('b010)
, FSM_STATE_REQ  = STATE_W'('b011)
, FSM_STATE_WAIT = STATE_W'('b100)
} fsm_state_t;

logic                                   fsm_state_en;
`Q_DFFRE(fsm_state_t, fsm_state, fsm_state_en, FSM_STATE_IDLE, clk);
logic                                   fsm_state_engid_en;
`Q_DFFE(stk_pkg::engid_t, fsm_state_engid, fsm_state_engid_en, clk);
logic [cfg_pkg::ENGS_N - 1:0]           fsm_state_engid_d;
//
logic                                   fsm_state_idle;
logic                                   fsm_state_init;
logic                                   fsm_state_req;
logic                                   fsm_state_wait;
//
fsm_state_t                             fsm_state_idle_next;
fsm_state_t                             fsm_state_init_next;
fsm_state_t                             fsm_state_req_next;
fsm_state_t                             fsm_state_wait_next;
//
logic                                   cmd_queue_pop;
stk_pkg::engid_t                        cmd_queue_pop_dat;
logic                                   cmd_queue_push;
stk_pkg::engid_t                        cmd_queue_push_dat;
`Q_DFFR(logic, cmd_queue_full, 1'b0, clk);
`Q_DFFR(logic, cmd_queue_empty, 1'b1, clk);
logic                                   inv_req_vld;

// Issue Interface:
//
logic                                   inv_start_evt;
logic                                   inv_end_evt;
logic                                   iss_req;
logic                                   rsp_inv_kill;
logic [cfg_pkg::ENGS_N - 1:0]           active_set_d;
logic [cfg_pkg::ENGS_N - 1:0]           active_clr_d;

// Response Interface:
//
logic                                   wrbk_hit;

// ========================================================================== //
//                                                                            //
//  Opcode Queue                                                              //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Enqueue
assign cmd_queue_push = i_push;

// -------------------------------------------------------------------------- //
// Dequeue
assign cmd_queue_push_dat = i_push_dat;
assign cmd_queue_pop = inv_start_evt;

// -------------------------------------------------------------------------- //
//
queue_rf #(.N(2), .W(stk_pkg::ENGID_W)) u_cmd_queue (
//
  .i_push                     (cmd_queue_push)
, .i_push_dat                 (cmd_queue_push_dat)
//
, .i_pop                      (cmd_queue_pop)
, .o_pop_dat                  (cmd_queue_pop_dat)
//
, .o_full_w                   (cmd_queue_full_w)
, .o_empty_w                  (cmd_queue_empty_w)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

// ========================================================================== //
//                                                                            //
//  FSM State                                                                 //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign fsm_state_engid_en = inv_start_evt;
assign fsm_state_engid_w = cmd_queue_pop_dat;

// ========================================================================== //
//                                                                            //
//  FSM State Transition Logic                                                //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// State decoders
assign fsm_state_idle = (fsm_state_r == FSM_STATE_IDLE);
assign fsm_state_init = (fsm_state_r == FSM_STATE_INIT);
assign fsm_state_req = (fsm_state_r == FSM_STATE_REQ);
assign fsm_state_wait = (fsm_state_r == FSM_STATE_WAIT);

// -------------------------------------------------------------------------- //
// State IDLE
assign fsm_state_idle_next =
  (~cmd_queue_empty_r) ? FSM_STATE_INIT : FSM_STATE_IDLE;

// -------------------------------------------------------------------------- //
// State INIT
assign fsm_state_init_next = FSM_STATE_REQ;

// -------------------------------------------------------------------------- //
// State REQ
assign fsm_state_req_next = i_iss_ack ? FSM_STATE_WAIT : FSM_STATE_REQ;

// -------------------------------------------------------------------------- //
// State WAIT
assign fsm_state_wait_next =
  (wrbk_hit
    ? (i_wrbk_uc_islast_r ? FSM_STATE_IDLE : FSM_STATE_REQ)
    : FSM_STATE_WAIT);

// -------------------------------------------------------------------------- //
//
assign fsm_state_w =
    ({STATE_W{fsm_state_idle}} & fsm_state_idle_next)
  | ({STATE_W{fsm_state_init}} & fsm_state_init_next)
  | ({STATE_W{fsm_state_req}}  & fsm_state_req_next)
  | ({STATE_W{fsm_state_wait}} & fsm_state_wait_next);

assign fsm_state_en = (fsm_state_w != fsm_state_r);

// ========================================================================== //
//                                                                            //
//  Issue Interface                                                           //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign inv_start_evt = fsm_state_init;


assign iss_req = fsm_state_req;

dec #(.W(cfg_pkg::ENGS_N)) u_dec (
  .i_x(fsm_state_engid_r), .o_y(fsm_state_engid_d)
);

//
assign active_set_d = {cfg_pkg::ENGS_N{inv_start_evt}} & fsm_state_engid_d;

//
assign active_clr_d = {cfg_pkg::ENGS_N{inv_end_evt}} & fsm_state_engid_d;


// -------------------------------------------------------------------------- //
//

assign inv_end_evt =
  fsm_state_wait & (wrbk_hit & i_wrbk_uc_islast_r);

assign rsp_inv_kill =
  fsm_state_wait & wrbk_hit & (~i_wrbk_uc_islast_r);

// ========================================================================== //
//                                                                            //
//  Response Interface                                                        //
//                                                                            //
// ========================================================================== //

assign wrbk_hit = i_wrbk_uc_vld_r & (i_wrbk_uc_engid_r == fsm_state_engid_r);

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign o_full_r = cmd_queue_full_r;

assign o_iss_req = iss_req;
assign o_iss_engid = fsm_state_engid_r;
assign o_active_set_d = active_set_d;
assign o_active_clr_d = active_clr_d;
assign o_rsp_inv_kill = rsp_inv_kill;

endmodule : stk_pipe_ad_inv
