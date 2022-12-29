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

module tb_stk (

// -------------------------------------------------------------------------- //
// Command Interface 0
  input wire logic                                i_cmd0_vld
, input wire stk_pkg::opcode_t                    i_cmd0_opcode
, input wire logic [127:0]                        i_cmd0_dat
//
, output wire logic                               o_cmd0_ack

// -------------------------------------------------------------------------- //
// Command Interface 1
, input wire logic                                i_cmd1_vld
, input wire stk_pkg::opcode_t                    i_cmd1_opcode
, input wire logic [127:0]                        i_cmd1_dat
//
, output wire logic                               o_cmd1_ack

// -------------------------------------------------------------------------- //
// Command Interface 2
, input wire logic                                i_cmd2_vld
, input wire stk_pkg::opcode_t                    i_cmd2_opcode
, input wire logic [127:0]                        i_cmd2_dat
//
, output wire logic                               o_cmd2_ack

// -------------------------------------------------------------------------- //
// Command Interface 3
, input wire logic                                i_cmd3_vld
, input wire stk_pkg::opcode_t                    i_cmd3_opcode
, input wire logic [127:0]                        i_cmd3_dat
//
, output wire logic                               o_cmd3_ack

// -------------------------------------------------------------------------- //
// Response Interface
, output wire logic [cfg_pkg::ENGS_N - 1:0]       o_rsp_vld
, output wire logic [127:0]                       o_rsp_dat
, output wire stk_pkg::status_t                   o_rsp_status

// -------------------------------------------------------------------------- //
// Testbench State
, output wire logic [31:0]                        o_tb_cycle
//
, output wire logic                               o_busy_r
//
, output wire logic                               o_lk_vld_w
, output wire stk_pkg::engid_t                    o_lk_engid_w
, output wire stk_pkg::opcode_t                   o_lk_opcode_w
, output wire logic [127:0]                       o_lk_dat_w

, output wire logic                               o_empty0_r
, output wire logic                               o_empty1_r
, output wire logic                               o_empty2_r
, output wire logic                               o_empty3_r

, output wire logic                               o_full_r

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

int                                             tb_cycle;
logic [cfg_pkg::ENGS_N - 1:0]                   cmd_vld;
stk_pkg::opcode_t [cfg_pkg::ENGS_N - 1:0]       cmd_opcode;
logic [cfg_pkg::ENGS_N - 1:0][127:0]            cmd_dat;
logic [cfg_pkg::ENGS_N - 1:0]                   cmd_ack;

// ========================================================================== //
//                                                                            //
//  Command Interface (In)                                                    //
//                                                                            //
// ========================================================================== //

assign cmd_opcode [0] = i_cmd0_opcode;
assign cmd_opcode [1] = i_cmd1_opcode;
assign cmd_opcode [2] = i_cmd2_opcode;
assign cmd_opcode [3] = i_cmd3_opcode;

assign cmd_vld [0] = i_cmd0_vld;
assign cmd_vld [1] = i_cmd1_vld;
assign cmd_vld [2] = i_cmd2_vld;
assign cmd_vld [3] = i_cmd3_vld;

assign cmd_dat [0] = i_cmd0_dat;
assign cmd_dat [1] = i_cmd1_dat;
assign cmd_dat [2] = i_cmd2_dat;
assign cmd_dat [3] = i_cmd3_dat;

// ========================================================================== //
//                                                                            //
//  Unit Under Test (UUT)                                                     //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
stk u_stk (
//
  .i_cmd_vld                            (cmd_vld)
, .i_cmd_opcode                         (cmd_opcode)
, .i_cmd_dat                            (cmd_dat)
//
, .o_cmd_ack                            (cmd_ack)
//
, .o_rsp_vld                            (o_rsp_vld)
, .o_rsp_dat                            (o_rsp_dat)
, .o_rsp_status                         (o_rsp_status)
//
, .clk                                  (clk)
, .arst_n                               (arst_n)
);

// ========================================================================== //
//                                                                            //
//  TB                                                                        //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
initial tb_cycle = 0;

always_ff @(posedge clk)
  tb_cycle <= tb_cycle + 'b1;

// ========================================================================== //
//                                                                            //
//  Hierarchical Probes                                                       //
//                                                                            //
// ========================================================================== //

// Allocation Stage, Initialization Logic busy flag.
//
assign o_busy_r = u_stk.u_stk_pipe.u_stk_pipe_al.u_stk_pipe_al_init.busy_r;

assign o_lk_vld_w = u_stk.u_stk_pipe.u_stk_pipe_ad.o_lk_vld_w;
assign o_lk_engid_w = u_stk.u_stk_pipe.u_stk_pipe_ad.o_lk_engid_w;
assign o_lk_opcode_w = u_stk.u_stk_pipe.u_stk_pipe_ad.o_lk_opcode_w;
assign o_lk_dat_w = u_stk.u_stk_pipe.u_stk_pipe_ad.o_lk_dat_w;

assign o_empty0_r = u_stk.u_stk_pipe.u_stk_pipe_lk.empty_r [0];
assign o_empty1_r = u_stk.u_stk_pipe.u_stk_pipe_lk.empty_r [1];
assign o_empty2_r = u_stk.u_stk_pipe.u_stk_pipe_lk.empty_r [2];
assign o_empty3_r = u_stk.u_stk_pipe.u_stk_pipe_lk.empty_r [3];

// NOTE: terminology here is inverted; STK is full, if no further slots
// exist; STK is empty, if the allocation unit is full.
assign o_full_r = u_stk.u_stk_pipe.u_stk_pipe_al.o_ad_empty_r;

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

assign o_tb_cycle = tb_cycle;

assign o_cmd0_ack = cmd_ack [0];
assign o_cmd1_ack = cmd_ack [1];
assign o_cmd2_ack = cmd_ack [2];
assign o_cmd3_ack = cmd_ack [3];

endmodule : tb_stk
