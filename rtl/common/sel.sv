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
`include "macros.vh"

module sel #(
// Width of selection vector
  parameter int              W
// Number of selection bits
, parameter int              N

, localparam int             OUT_W = (W / N);
) (
// -------------------------------------------------------------------------- //
// Selection Vector
  input wire logic [W - 1:0]                     i_x
//
, input wire logic [$clog(N) - 1:0]              i_sel

// -------------------------------------------------------------------------- //
// Encoded output
, output wire logic [OUT_W - 1:0]                o_y
);

logic [N - 1:0][OUT_W - 1:0]            bins;
logic [OUT_W - 1:0]                     y;

// -------------------------------------------------------------------------- //
//
for (genvar i = 0; i < N; i++) begin

assign bins [i] = i_x [(OUT_W * i) +: OUT_W];

end

mux #(.N, .W(OUT_W)) u_idx_mux (.i_x(bins), .i_sel, .o_y(y));

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

assign o_y = y;

endmodule : sel
