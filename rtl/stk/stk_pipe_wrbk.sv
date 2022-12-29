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

module stk_pipe_wrbk (
// -------------------------------------------------------------------------- //
// Writeback ("WRBK") microcode
  input wire logic                                i_wrbk_uc_vld_r
, input wire stk_pkg::engid_t                     i_wrbk_uc_engid_r
, input wire stk_pkg::status_t                    i_wrbk_uc_status_r
, input wire logic [127:0]                        i_wrbk_uc_dat_r
//
, input wire logic                                i_wrbk_rsp_inv_kill

// -------------------------------------------------------------------------- //
// Response
, output wire logic [cfg_pkg::ENGS_N - 1:0]       o_rsp_vld
, output wire logic [127:0]                       o_rsp_dat
, output wire stk_pkg::status_t                   o_rsp_status
);

// ========================================================================== //
//                                                                            //
//  Wires                                                                     //
//                                                                            //
// ========================================================================== //

// Response Interface
//
logic [cfg_pkg::ENGS_N - 1:0]                     engid_d;
logic                                             rsp_emit_vld;
logic [cfg_pkg::ENGS_N - 1:0]                     rsp_vld;

// ========================================================================== //
//                                                                            //
//  Response Interface                                                        //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign rsp_emit_vld = i_wrbk_uc_vld_r & (~i_wrbk_rsp_inv_kill);

dec #(.W(stk_pkg::BANKS_N)) u_rsp_dec (
  .i_x(i_wrbk_uc_engid_r), .o_y(engid_d)
);

assign rsp_vld = ({stk_pkg::BANKS_N{rsp_emit_vld}} & engid_d);

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign o_rsp_vld = rsp_vld;
assign o_rsp_dat = i_wrbk_uc_dat_r;
assign o_rsp_status = i_wrbk_uc_status_r;

endmodule : stk_pipe_wrbk
