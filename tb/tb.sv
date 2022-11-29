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

module tb (
// -------------------------------------------------------------------------- //
//
  input wire logic                                i_ingress_vld
, input wire logic                                i_ingress_sop
, input wire logic                                i_ingress_eop

// -------------------------------------------------------------------------- //
, output wire logic                               o_egress_vld_r
, output wire logic                               o_egress_sop_r
, output wire logic                               o_egress_eop_r

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
  .i_ingress_vld                       (i_ingress_vld)
, .i_ingress_sop                       (i_ingress_sop)
, .i_ingress_eop                       (i_ingress_eop)
//
, .o_egress_vld_r                      (o_egress_vld_r)
, .o_egress_sop_r                      (o_egress_sop_r)
, .o_egress_eop_r                      (o_egress_eop_r)
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
