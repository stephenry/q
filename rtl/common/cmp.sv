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

module cmp #(
  // Width of comparitor
  parameter int W
  // Signed/Unsigned comparison
, parameter bit IS_SIGNED = 'b1
  // Allow synthesis to infer comparison logic
, parameter bit FPGA_INFER = 'b0
) (
// -------------------------------------------------------------------------- //
//
  input wire logic [W - 1:0]                      i_a
, input wire logic [W - 1:0]                      i_b
//
, output wire logic                               o_eq
, output wire logic                               o_gt
, output wire logic                               o_lt
);

// ========================================================================== //
//                                                                            //
//  Wires                                                                     //
//                                                                            //
// ========================================================================== //

logic                                   eq;
logic                                   gt;
logic                                   lt;

// ========================================================================== //
//                                                                            //
//  Combinatorial Logic                                                       //
//                                                                            //
// ========================================================================== //

if (!FPGA_INFER) begin

logic [W - 1:0]                         cla_a;
logic [W - 1:0]                         cla_b;
logic                                   cla_cin;
logic [W - 1:0]                         cla_y;
logic                                   cla_cout;

// Flags:
logic                                   z;
logic                                   v;
logic                                   n;
logic                                   c;

// -------------------------------------------------------------------------- //
//
assign cla_a = i_a;
assign cla_b = ~i_b;
assign cla_cin = 1'b1;

cla #(.W(W)) u_cla (
  //
    .i_a                                (cla_a)
  , .i_b                                (cla_b)
  , .i_cin                              (cla_cin)
  //
  , .o_y                                (cla_y)
  , .o_cout                             (cla_cout)
);


// -------------------------------------------------------------------------- //
// Derive flags

// Overflow flag (sign at output differs from signs at input)
//
assign v = ( cla_a[W - 1] &  cla_b[W - 1] & ~cla_y[W - 1]) |
           (~cla_a[W - 1] & ~cla_b[W - 1] &  cla_y[W - 1]);

// Zero flag (operands are equal)
//
assign z = (cla_y == '0);

// Negative flag (result was < 0 in signed-case)
//
assign n = cla_y[W - 1];

// Carry out
//
assign c = cla_cout;

// -------------------------------------------------------------------------- //
// Compute final status
//
assign eq = z;
assign lt = IS_SIGNED ? (n ^ v) : c;
assign gt = IS_SIGNED ? ~(z | (n ^ v)) : ~(c | z);

end else begin // if (FPGA_INFER)

// Otherwise, simply allow synthesis to infer comparison operators. In an FPGA
// setting, this case result in superior QOR if specialized adder hard-macros
// are inferred instead of random logic.

assign eq = (i_a == i_b);
assign lt = IS_SIGNED ? ($signed(i_a) < $signed(i_b)) : (i_a < i_b);
assign gt = IS_SIGNED ? ($signed(i_a) > $signed(i_b)) : (i_a > i_b);

end // else: !if(FPGA_INFER)

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

assign o_eq = eq;
assign o_gt = gt;
assign o_lt = lt;

endmodule // cmp
