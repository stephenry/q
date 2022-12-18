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

`ifndef Q_TECH_COMMON_MACROS_VH
`define Q_TECH_COMMON_MACROS_VH

`define Q_ICG(__gated_clk, __clk, __en) \
  icg u_``__gate_clk``_icg (\
    .o_clk_gated(__gated_clk), .i_clk(__clk), .en(__en), .i_dft_en(1'b0))

`define Q_DFF(__type, __name, __clk) \
    __type __name``_r; \
    __type __name``_w; \
    dff #(.W($bits(__type))) u_``__name``_reg ( \
      .d(__name``_w), .q(__name``_r), .clk(__clk))

`define Q_DFFE(__type, __name, __en, __clk) \
    __type __name``_r; \
    __type __name``_w; \
    dffe #(.W($bits(__type))) u_``__name``_reg ( \
      .d(__name``_w), .q(__name``_r), .en(__en), .clk(__clk))

`define Q_DFFR(__type, __name, __init, __clk) \
    __type __name``_r; \
    __type __name``_w; \
    dffr #(.W($bits(__type)), .INIT(__init)) u_``__name``_reg ( \
      .d(__name``_w), .q(__name``_r), .arst_n, .clk(__clk))

`define Q_DFFRE(__type, __name, __en, __init, __clk) \
    __type __name``_r; \
    __type __name``_w; \
    dffre #(.W($bits(__type)), .INIT(__init)) u_``__name``_reg ( \
      .d(__name``_w), .q(__name``_r), .en(__en), .arst_n, .clk(__clk))

`endif
