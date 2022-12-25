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
#include "tb_stk/tb_stk_main.h"
#include "tb_stk/tb_stk_smoke.h"
#include "test.h"
#include "sim.h"
#include <memory>
#include <string>

using namespace tb_stk;

struct StkTest::Factory {
  static std::unique_ptr<StkTest> construct(const std::string& name) {
    std::unique_ptr<StkTest> p;
    if (Globals::test_name == "tb_stk_smoke") {
      p = std::make_unique<smoke::Test>();
    }
    return p;
  }
};

class StkTest::Builder : public TestBuilder {
public:
  explicit Builder() = default;

  std::unique_ptr<Test> construct() override {
    std::unique_ptr<StkTest> t{
      StkTest::Factory::construct(Globals::test_name)};
    t->kernel_ = std::make_unique<KernelVerilated<Vtb_stk, Driver>>();
    t->model_ = std::make_unique<Model>();
    return t;
  }

};

namespace tb_stk {

void init(TestRegistry& tr) {
  tr.add<StkTest::Builder>("tb_stk_smoke");
}

} // namespace tb_stk
