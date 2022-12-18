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

module stk_pipe_al_init (

// -------------------------------------------------------------------------- //
// Memory Interfacex
  output wire logic                               o_init_wen_r
, output wire stk_pkg::line_id_t                  o_init_waddr_r
, output wire logic [127:0]                       o_init_wdata_r

// -------------------------------------------------------------------------- //
// Control/Status
, input wire logic                                i_init
//
, output wire logic                               o_busy_r

// -------------------------------------------------------------------------- //
// Clk/Reset
, input wire logic                                clk
);

// ========================================================================== //
//                                                                            //
//  Wires                                                                     //
//                                                                            //
// ========================================================================== //

typedef enum logic [2:0] {
  // Transition state on initialization to perform
  // the necessary reset of FSM state.
  FSM_STATE_IDLE = 3'b001,
  // Active state where memory is being initialized.
  FSM_STATE_BUSY = 3'b010,
  // State indicating operation has completed.
  FSM_STATE_DONE = 3'b100,
  // Fall off the earth into nothingness state
  // once operation has been completed.
  FSM_STATE_EXIT = 3'b000
 } fsm_state_t;
localparam int FSM_STATE_W = $bits(fsm_state_t);


fsm_state_t                             fsm_state_next;
fsm_state_t                             fsm_state_idle_next;
fsm_state_t                             fsm_state_busy_next;
fsm_state_t                             fsm_state_done_next;
logic                                   fsm_state_en;
logic                                   st_idle;
logic                                   st_busy;
logic                                   st_done;
logic                                   init_waddr_en;
logic                                   init_waddr_is_final;

// ========================================================================== //
//                                                                            //
//  Flops                                                                     //
//                                                                            //
// ========================================================================== //

`Q_DFF(logic, busy, clk);
`Q_DFF(logic, init_wen, clk);
`Q_DFFE(stk_pkg::line_id_t, init_waddr, init_waddr_en, clk);
`Q_DFFE(fsm_state_t, fsm_state, fsm_state_en, clk);

// ========================================================================== //
//                                                                            //
//  Combinatorial Logic                                                       //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// State decoders

assign st_idle = (fsm_state_r == FSM_STATE_IDLE);
assign st_busy = (fsm_state_r == FSM_STATE_BUSY);
assign st_done = (fsm_state_r == FSM_STATE_DONE);

// -------------------------------------------------------------------------- //
// Status:
assign busy_w =
    (fsm_state_w == FSM_STATE_IDLE) |
    (fsm_state_w == FSM_STATE_BUSY) |
    (fsm_state_w == FSM_STATE_DONE);

// -------------------------------------------------------------------------- //
// Address:

// Initialize address at start or throughout operation.
assign init_waddr_en = (st_busy | st_idle);

// Address update; zero in IDLE state, otherwise always increment.
assign init_waddr_w  = st_idle ? '0 : (init_waddr_r + 'b1);

localparam int LAST_ADDR = stk_pkg::C_BANK_LINES_N - 1;

assign init_waddr_is_final = (init_waddr_r == stk_pkg::line_id_t'(LAST_ADDR));

assign init_wen_w = (fsm_state_w == FSM_STATE_BUSY);

// -------------------------------------------------------------------------- //
// State transitions:

assign fsm_state_en = i_init | (fsm_state_r != fsm_state_w);

assign fsm_state_idle_next = FSM_STATE_BUSY;

// Remain in BUSY state until entire address range has been exhausted.
assign fsm_state_busy_next =
    init_waddr_is_final ? FSM_STATE_DONE : FSM_STATE_BUSY;

assign fsm_state_done_next = FSM_STATE_EXIT;

// State update mux.
assign fsm_state_next =
    ({FSM_STATE_W{st_idle}} & fsm_state_idle_next) |
    ({FSM_STATE_W{st_busy}} & fsm_state_busy_next) |
    ({FSM_STATE_W{st_done}} & fsm_state_done_next) ;

assign fsm_state_w =
    ({FSM_STATE_W{ i_init}} & FSM_STATE_IDLE) |
    ({FSM_STATE_W{~i_init}} & fsm_state_next);

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

assign o_busy_r = busy_r;

assign o_init_wen_r = init_wen_r;
assign o_init_waddr_r = init_waddr_r;
assign o_init_wdata_r = '0;

endmodule : stk_pipe_al_init

`include "unmacros.vh"
