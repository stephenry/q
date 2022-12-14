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

module stk (
// -------------------------------------------------------------------------- //
//
  input wire logic [cfg_pkg::ENGS_N - 1:0]        i_cmd_vld
, input wire stk_pkg::opcode_t [cfg_pkg::ENGS_N - 1:0]
                                                  i_cmd_opcode
, input wire logic [cfg_pkg::ENGS_N - 1:0][127:0] i_cmd_dat
//
, output wire logic [cfg_pkg::ENGS_N - 1:0]       o_cmd_ack
//
, output wire logic [cfg_pkg::ENGS_N - 1:0]       o_rsp_vld
, output wire logic [127:0]                       o_rsp_dat
, output wire stk_pkg::status_t                   o_rsp_status

// -------------------------------------------------------------------------- //
// Clk/Reset
, input wire logic                                clk
, input wire logic                                arst_n
);

// ========================================================================== //
//                                                                            //
//  Instances                                                                 //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
stk_pipe u_stk_pipe (
//
  .i_cmd_vld                  (i_cmd_vld)
, .i_cmd_opcode               (i_cmd_opcode)
, .i_cmd_dat                  (i_cmd_dat)
, .o_cmd_ack                  (o_cmd_ack)
//
, .o_rsp_vld                  (o_rsp_vld)
, .o_rsp_dat                  (o_rsp_dat)
, .o_rsp_status               (o_rsp_status)
//
, .clk                        (clk)
, .arst_n                     (arst_n)
);

endmodule : stk
