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

// Leading Zero Detector (conditionally inverted with DETECT_ZERO)
//
//  i_x                          i_y
// ---------------------------------------------------------------
//  00000000000000000000         10000000000000000000
//  11111111111111111111         00000000000000000000
//  11111111110000000000         00000000001000000000
//  11110101010101010101         00001000000000000000

module lzd #(
  // Width of vector
  parameter int W
  // Leading from LSB or MSB
, parameter bit FROM_LSB = 'b0
  // Detect first 'b0 (or 'b1 in !DETECT_ZERO case)
, parameter bit DETECT_ZERO = 'b0
) (
// -------------------------------------------------------------------------- //
//
  input wire logic [W - 1:0]                      i_x

// -------------------------------------------------------------------------- //
//
, output wire logic [W - 1:0]                     o_y
);

// ========================================================================== //
//                                                                            //
//  Wires                                                                     //
//                                                                            //
// ========================================================================== //

logic [W - 1:0]                         x_to_first_one;
logic [W - 1:0]                         y;

// ========================================================================== //
//                                                                            //
//  Combinatorial Logic                                                       //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
// Downstream logic detects first 'b1, therefore conditionally invert input
// vector to correct format.
//
assign x_to_first_one = {W{DETECT_ZERO}} ^ i_x;

// ========================================================================== //
//                                                                            //
//  Instances                                                                 //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
pri #(.W(W), .FROM_LSB(FROM_LSB)) u_pri (
//
  .i_x                                  (x_to_first_one)
//
, .o_y                                  (y)
);

// ========================================================================== //
//                                                                            //
//  Outputs                                                                   //
//                                                                            //
// ========================================================================== //

// -------------------------------------------------------------------------- //
//
assign o_y = y;

endmodule : lzd
