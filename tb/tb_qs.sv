//========================================================================== //
// Copyright (c) 2020, Stephen Henry
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

`timescale 1ns/1ps

`default_nettype none

`include "tb_qs_pkg.vh"
`include "qs_pkg.vh"
`include "qs_srt_pkg.vh"

module tb_qs (

  // ======================================================================== //
  // Command Interface

  // Unsorted Input:
    input                                         in_vld
  , input                                         in_sop
  , input                                         in_eop
  , input [tb_qs_pkg::OPT_W - 1:0]                in_dat

  , output                                        in_rdy_r

  // Sorted output:
  , output logic                                  out_vld_r
  , output logic                                  out_sop_r
  , output logic                                  out_eop_r
  , output logic                                  out_err_r
  , output logic [tb_qs_pkg::OPT_W - 1:0]         out_dat_r

  // ======================================================================== //
  // Micro-code probes

  // Disassembly interface:
  , output logic                                  uc_inst_commit
  , output qs_srt_pkg::inst_t                     uc_inst
  , output qs_srt_pkg::pc_t                       uc_inst_pc
  // Writeback:
  , output logic                                  uc_rf_wen
  , output qs_srt_pkg::reg_t                      uc_rf_wa
  , output qs_pkg::w_t                            uc_rf_wdata
  // Flags:
  , output logic                                  uc_flags_en
  , output logic                                  uc_flags_eq
  , output logic                                  uc_flags_lt

  // ======================================================================== //
  // TB support
  , output logic [63:0]                           tb_cycle

  // ======================================================================== //
  // Clk/Reset
  , input                                         clk
  , input                                         rst
);

  // ------------------------------------------------------------------------ //
  //
  initial tb_cycle  = '0;

  always_ff @(posedge clk)
    tb_cycle += 'b1;

  // ------------------------------------------------------------------------ //
  //
  qs u_qs (
    //
      .in_vld                 (in_vld                  )
    , .in_sop                 (in_sop                  )
    , .in_eop                 (in_eop                  )
    , .in_dat                 (in_dat                  )
    , .in_rdy_r               (in_rdy_r                )
    //
    , .out_vld_r              (out_vld_r               )
    , .out_sop_r              (out_sop_r               )
    , .out_eop_r              (out_eop_r               )
    , .out_err_r              (out_err_r               )
    , .out_dat_r              (out_dat_r               )
    //
    , .clk                    (clk                     )
    , .rst                    (rst                     )
  );

  // ------------------------------------------------------------------------ //
  //
  always_comb begin : uc_probes_PROC

    // Probe instruction at execution stage.
    uc_inst_commit = u_qs.u_qs_srt.ca_commit;
    uc_inst        = u_qs.u_qs_srt.ca_ucode_r.inst;
    uc_inst_pc     = u_qs.u_qs_srt.ca_ucode_r.pc;

    // Probe writebacks to register file.
    uc_rf_wen      = u_qs.u_qs_srt.rf_wen;
    uc_rf_wa       = u_qs.u_qs_srt.rf_wa;
    uc_rf_wdata    = u_qs.u_qs_srt.rf_wdata;

    // Probe to check flags.
    uc_flags_en    = u_qs.u_qs_srt.ca_ucode_r.flags_en;
    uc_flags_eq    = u_qs.u_qs_srt.ar_flags_r.eq;
    uc_flags_lt    = u_qs.u_qs_srt.ar_flags_r.lt;

  end // block: uc_probes_PROC

endmodule // tb_qs
