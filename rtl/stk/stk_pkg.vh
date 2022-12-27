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

`ifndef Q_RTL_STK_STK_PKG_VH
`define Q_RTL_STK_STK_PKG_VH

`include "cfg_pkg.vh"

package stk_pkg;

// ========================================================================== //
//                                                                            //
//  Opcodes                                                                   //
//                                                                            //
// ========================================================================== //

localparam int OPCODE_W = 2;

typedef enum logic [OPCODE_W - 1:0] {
  OPCODE_NOP  = OPCODE_W'('b00)
, OPCODE_PUSH = OPCODE_W'('b01)
, OPCODE_POP  = OPCODE_W'('b10)
, OPCODE_INV  = OPCODE_W'('b11)
} opcode_t;



// ========================================================================== //
//                                                                            //
//  Status                                                                    //
//                                                                            //
// ========================================================================== //

localparam int STATUS_W = 2;

typedef enum logic [STATUS_W - 1:0] {
  STATUS_OKAY     = STATUS_W'('b00)
, STATUS_ERRFULL  = STATUS_W'('b10)
, STATUS_ERREMPTY = STATUS_W'('b11)
} status_t;

// ========================================================================== //
//                                                                            //
//  Foo.                                                                      //
//                                                                            //
// ========================================================================== //

localparam ENGID_W = $clog2(cfg_pkg::ENGS_N);
typedef logic [ENGID_W - 1:0] engid_t;


localparam int BANKS_N = 4;

// Constant; to change, associated SRAM must be resized.
localparam int C_BANK_LINES_N = 1024;

localparam int BANK_W = $clog2(BANKS_N);

typedef logic [BANK_W - 1:0] bank_id_t; // -> bankid_t

localparam int BANK_LINE_OFFSET_W = $clog2(C_BANK_LINES_N);

typedef logic [BANK_LINE_OFFSET_W - 1:0] line_id_t;

typedef struct packed {
  bank_id_t         bnk_id;
  line_id_t         line_id;
} ptr_t;

localparam int PTR_W = $bits(ptr_t);

// TODO(stephenry): invert
localparam int LINE_ID_W = $bits(line_id_t);

endpackage // stk_pkg

`endif
