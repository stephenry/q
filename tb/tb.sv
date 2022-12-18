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

`include "cfg_pkg.vh"

module tb (
// -------------------------------------------------------------------------- //
// Master Interface
  input wire logic                                i_mst_rsp_vld
, input wire logic                                i_mst_rsp_sop
, input wire logic                                i_mst_rsp_eop
, input wire logic [127:0]                        i_mst_rsp_dat

, output wire logic                               o_mst_cmd_vld
, output wire logic                               o_mst_cmd_sop
, output wire logic                               o_mst_cmd_eop
, output wire logic [127:0]                       o_mst_cmd_dat

// -------------------------------------------------------------------------- //
// Slave Interface
, input wire logic                                i_slv_cmd_vld
, input wire logic                                i_slv_cmd_rnw
, input wire logic [63:0]                         i_slv_cmd_dat
//
, output wire logic                               o_slv_rsp_vld
, output wire logic [63:0]                        o_slv_rsp_dat

// -------------------------------------------------------------------------- //
// Testbench State
, output wire logic [31:0]                        o_tb_cycle

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

int                                     tb_cycle;

// ========================================================================== //
//                                                                            //
//  UUT                                                                       //
//                                                                            //
// ========================================================================== //

q u_q (
//
  .i_mst_rsp_vld                       (i_mst_rsp_vld)
, .i_mst_rsp_sop                       (i_mst_rsp_sop)
, .i_mst_rsp_eop                       (i_mst_rsp_eop)
, .i_mst_rsp_dat                       (i_mst_rsp_dat)
//
, .o_mst_cmd_vld                       (o_mst_cmd_vld)
, .o_mst_cmd_sop                       (o_mst_cmd_sop)
, .o_mst_cmd_eop                       (o_mst_cmd_eop)
, .o_mst_cmd_dat                       (o_mst_cmd_dat)
//
, .i_slv_cmd_vld                       (i_slv_cmd_vld)
, .i_slv_cmd_rnw                       (i_slv_cmd_rnw)
, .i_slv_cmd_dat                       (i_slv_cmd_dat)
//
, .o_slv_rsp_vld                       (o_slv_rsp_vld)
, .o_slv_rsp_dat                       (o_slv_rsp_dat)
//
, .clk                                 (clk)
, .arst_n                              (arst_n)
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
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

assign o_tb_cycle = tb_cycle;

endmodule : tb
