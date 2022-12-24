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
#include "tb_stk/tb_stk_smoke.h"
#include "tb_stk/Vobj/Vtb_stk.h"
#include "rnd.h"
#include "test.h"
#include "tb.h"
#include <exception>
#include <utility>

using namespace tb_stk;

StkDriver::StkDriver(Vtb_stk* tb_stk)
  : tb_stk_(tb_stk)
{}

void StkDriver::clk(bool c) {
  VSupport::logic(&tb_stk_->clk, c);
}

bool StkDriver::clk() const {
  return VSupport::logic(&tb_stk_->clk);
}

vluint64_t StkDriver::tb_cycle() const {
  return tb_stk_->o_tb_cycle;
}

void StkDriver::arst_n(bool c) {
  VSupport::logic(&tb_stk_->arst_n, c);
}

bool StkDriver::arst_n() const {
  return VSupport::logic(&tb_stk_->arst_n);
}

bool StkDriver::busy_r() const {
  return VSupport::logic(&tb_stk_->o_busy_r);
}


void StkDriver::issue(std::size_t ch, Opcode opcode) {
  auto to_underlying = [](Opcode opcode) {
    return static_cast<std::underlying_type_t<Opcode>>(opcode);
  };
  switch (ch) {
  case 0: tb_stk_->i_cmd0_opcode = to_underlying(opcode); break;
  case 1: tb_stk_->i_cmd1_opcode = to_underlying(opcode); break;
  case 2: tb_stk_->i_cmd2_opcode = to_underlying(opcode); break;
  case 3: tb_stk_->i_cmd3_opcode = to_underlying(opcode); break;
  }
}

void StkDriver::issue(std::size_t ch, Opcode opcode, VlWide<4>& dat) {
  issue(ch, opcode);
  switch (ch) {
  case 0: tb_stk_->i_cmd0_dat = dat; break;
  case 1: tb_stk_->i_cmd1_dat = dat; break;
  case 2: tb_stk_->i_cmd2_dat = dat; break;
  case 3: tb_stk_->i_cmd3_dat = dat; break;
  }
}

namespace {

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


};

namespace tb_stk {

class Model {
public:
  explicit Model() = default;

  bool empty(std::size_t ch) const { return stks_[ch].empty(); }

  void push(std::size_t ch, const VlWide<4>& w) {
    stks_[ch].push_back(w);
  }

  void pop(std::size_t ch, VlWide<4>& w) {
    w = stks_[ch].back();
    stks_[ch].pop_back();
  }

private:
  std::array<std::vector<VlWide<4>>, cfg::ENGS_N> stks_;
};

StkTest::StkTest() {
  model_ = std::make_unique<Model>();
}

StkTest::~StkTest() {}

void StkTest::issue(std::size_t ch, Opcode opcode) {
  struct IssueNonDataOpcode : Event {
    explicit IssueNonDataOpcode(std::size_t ch, Opcode opcode)
      : ch_(ch), opcode_(opcode)
    {}
    bool execute(StkDriver* d) override {
      d->issue(ch_, opcode_);
      return false;
    }
  private:
    std::size_t ch_;
    Opcode opcode_;
  };
  event_queue_.push_back(std::make_unique<IssueNonDataOpcode>(ch, opcode));
}

void StkTest::issue(std::size_t ch, Opcode opcode, const VlWide<4>& dat) {
  struct IssueEvent : Event {
    explicit IssueEvent(
      std::size_t ch, Opcode opcode, const VlWide<4>& dat)
      : ch_(ch), opcode_(opcode), dat_(dat)
    {}
    bool execute(StkDriver* d) override {
      d->issue(ch_, opcode_, dat_);
      return false;
    }
  private:
    std::size_t ch_;
    Opcode opcode_;
    VlWide<4> dat_;
  };
  event_queue_.push_back(std::make_unique<IssueEvent>(ch, opcode, dat));
}

void StkTest::wait(std::size_t cycles) {
  struct WaitCyclesEvent : Event {
    explicit WaitCyclesEvent(std::size_t cycles)
      : cycles_(cycles)
    {}
    bool execute(StkDriver*) override {
      return (--cycles_ == 0);
    }
  private:
    std::size_t cycles_;
  };
  event_queue_.push_back(std::make_unique<WaitCyclesEvent>(cycles));
}

void StkTest::wait_until(EventType et) {
  struct WaitUntilEvent : Event {
    explicit WaitUntilEvent(EventType et)
      : et_(et)
    {}
    bool execute(StkDriver* d) override {
      switch (et_) {
      case EventType::EndOfInitialization:
        // Wait until after 10th cycle before sampling busy to allow
        // time for the initialization process to have been kicked off.
        return (d->tb_cycle() < 10) || d->busy_r();
        break;
      }
    }
  private:
    EventType et_;
  };

  event_queue_.push_back(std::make_unique<WaitUntilEvent>(et));
}

bool StkTest::on_posedge_clk() {
  // TODO(stephenry): change to assertion; should never be reached.
  if (event_queue_.empty()) return false;

  if (!event_queue_.back()->execute(kernel_->driver())) {
    event_queue_.pop_back();
  }

  // If more events to go, reschedule, otherwise, try to reprogram
  // further stimulus, and then reschedule iff more arrives.
  //
  return !event_queue_.empty() || program();
}

bool StkTest::run() {
  return kernel_->run(this);
}

class StkTestFactory : public TestFactory {
public:
  explicit StkTestFactory() = default;

  std::unique_ptr<Test> construct() override {
    std::unique_ptr<tb_stk::smoke::Test> t;
    t->kernel_ = std::make_unique<KernelVerilated<Vtb_stk, StkDriver>>();
    t->model_ = std::make_unique<Model>();
    return t;
  }

};

void init(TestRegistry& tr) {
  tr.add<StkTestFactory>("tb_stk_smoke");
}

} // namespace tb_stk
