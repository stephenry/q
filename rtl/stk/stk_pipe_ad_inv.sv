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
, output wire logic                               o_iss_islast

// -------------------------------------------------------------------------- //
// Writeback Interface
, input wire logic                                i_wrbk_uc_vld_r
, input wire stk_pkg::engid_t                     i_wrbk_uc_engid_r
, input wire logic                                i_wrbk_uc_nxtlast_r
, input wire logic                                i_wrbk_uc_islast_r

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
} fsm_state_t;

logic                                   fsm_state_en;
`Q_DFFRE(fsm_state_t, fsm_state, fsm_state_en, FSM_STATE_IDLE, clk);

`Q_DFFR(logic, full, 1'b0, clk);
`Q_DFFR(logic, empty, 1'b1, clk);
logic                                   inv_req_vld;

// ========================================================================== //
//                                                                            //
//  Opcode Queue                                                              //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
queue_rf #(.N(2), .W(stk_pkg::ENGID_W)) u_cmd_queue (
//
  .i_push                     (i_push)
, .i_push_dat                 (i_push_dat)
//
, .i_pop                      ()
, .o_pop_dat                  ()
//
, .o_full_w                   (full_w)
, .o_empty_w                  (empty_w)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

// ========================================================================== //
//                                                                            //
//  FSM Logic                                                                 //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//


// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign o_full_r = full_r;

assign o_iss_req = 'b0;
assign o_iss_engid = '0;

endmodule : stk_pipe_ad_inv
