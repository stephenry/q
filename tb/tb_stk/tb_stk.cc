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

#include "tb_stk/tb_stk.h"
#include "tb_stk/Vobj/Vtb_stk.h"
#include "rnd.h"
#include "test.h"
#include "tb.h"
#include <exception>

namespace {

using namespace tb_stk;

struct StkDriver {

  static void clk(Vtb_stk* uut, bool c) {
    VSupport::logic(&uut->clk, c);
  }

  static bool clk(Vtb_stk* uut) {
    return VSupport::logic(&uut->clk);
  }

  static void arst_n(Vtb_stk* uut, bool c) {
    VSupport::logic(&uut->arst_n, c);
  }

  static bool arst_n(Vtb_stk* uut) {
    return VSupport::logic(&uut->arst_n);
  }

  static CData& cmd_opcode(Vtb_stk* uut, std::size_t ch) {
    switch (ch) {
    case 0: return uut->i_cmd0_opcode;
    case 1: return uut->i_cmd1_opcode;
    case 2: return uut->i_cmd2_opcode;
    case 3: return uut->i_cmd3_opcode;
    }
    throw std::invalid_argument("ch is out of range");
  }

  static VlWide<4>& cmd_dat(Vtb_stk* uut, std::size_t ch) {
    switch (ch) {
    case 0: return uut->i_cmd0_dat;
    case 1: return uut->i_cmd1_dat;
    case 2: return uut->i_cmd2_dat;
    case 3: return uut->i_cmd3_dat;
    }
    throw std::invalid_argument("ch is out of range");
  }

};

class StkTestCallBack : public KernelCallBack {
public:
  explicit StkTestCallBack(Vtb_stk* Vtb_stk)
    : Vtb_stk_(Vtb_stk)
  {}

  void cycles(std::size_t cycles) { cycles_ = cycles; }

  void idle() override {
  }

  bool on_negedge_clk() override {
    return (cycles_-- != 0);
  }

  bool on_posedge_clk() override {
    return true;
  }

private:
  std::size_t cycles_ = 0;
  Vtb_stk* Vtb_stk_;
};

class StkTestBasic : public Test {
public:
  explicit StkTestBasic() {
  }

  bool run() override {
    StkTestCallBack cb{k_.vtb()};
    cb.cycles(1 << 11);
    return k_.run(&cb);
  }

private:
  Kernel<Vtb_stk, StkDriver> k_;
};

class StkTestFactory : public TestFactory {
public:
  std::unique_ptr<Test> construct() override {
    return std::make_unique<StkTestBasic>();
  }
};

};

namespace tb_stk {

void init(TestRegistry& tr) {
  tr.add<StkTestFactory>("tb_stk");
}

} // namespace tb_stk
