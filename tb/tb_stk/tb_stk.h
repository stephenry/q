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
// ========================================================================== //

#ifndef Q_TB_STK_TB_STK_H
#define Q_TB_STK_TB_STK_H

#include "log.h"
#include "verilated.h"

// Forwards:
class TestRegistry;

namespace tb_stk {

enum class Opcode : CData {
  Nop    = 0b00,
  Push   = 0b01,
  Pop    = 0b10,
  Inv    = 0b11,
};

enum class Status : CData {
  Okay     = 0b00,
  ErrFull  = 0b10,
  ErrEmpty = 0b11,
};


void init(TestRegistry& tr);

} // namespace tb_stk

const char* to_string(tb_stk::Opcode opcode);

const char* to_string(tb_stk::Status Status);

template<>
struct StreamRenderer<tb_stk::Opcode> {
  static void write(std::ostream& os, tb_stk::Opcode opcode) {
    StreamRenderer<const char*>::write(os, to_string(opcode));
  }
};

template<>
struct StreamRenderer<tb_stk::Status> {
  static void write(std::ostream& os, tb_stk::Status status) {
    StreamRenderer<const char*>::write(os, to_string(status));
  }
};

#endif
