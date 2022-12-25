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

Driver::Driver(Vtb_stk* tb_stk)
  : tb_stk_(tb_stk)
{}

void Driver::clk(bool c) {
  VSupport::logic(&tb_stk_->clk, c);
}

bool Driver::clk() const {
  return VSupport::logic(&tb_stk_->clk);
}

vluint64_t Driver::tb_cycle() const {
  return tb_stk_->o_tb_cycle;
}

void Driver::arst_n(bool c) {
  VSupport::logic(&tb_stk_->arst_n, c);
}

bool Driver::arst_n() const {
  return VSupport::logic(&tb_stk_->arst_n);
}

bool Driver::busy_r() const {
  return VSupport::logic(&tb_stk_->o_busy_r);
}

void Driver::idle() {
  for (std::size_t ch = 0; ch < cfg::ENGS_N; ch++) {
    issue(ch, Opcode::Nop);
  }
}

void Driver::issue(std::size_t ch, Opcode opcode) {
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

void Driver::issue(std::size_t ch, Opcode opcode, VlWide<4>& dat) {
  issue(ch, opcode);
  switch (ch) {
  case 0: tb_stk_->i_cmd0_dat = dat; break;
  case 1: tb_stk_->i_cmd1_dat = dat; break;
  case 2: tb_stk_->i_cmd2_dat = dat; break;
  case 3: tb_stk_->i_cmd3_dat = dat; break;
  }
}

bool Driver::ack(std::size_t ch) const {
  switch (ch) {
  case 0: return VSupport::logic(&tb_stk_->o_cmd0_ack);
  case 1: return VSupport::logic(&tb_stk_->o_cmd1_ack);
  case 2: return VSupport::logic(&tb_stk_->o_cmd2_ack);
  case 3: return VSupport::logic(&tb_stk_->o_cmd3_ack);
  }
  return false;
}

namespace tb_stk {

StkTest::StkTest() {
  model_ = std::make_unique<Model>();
}

StkTest::~StkTest() {}

void StkTest::issue(std::size_t ch, Opcode opcode) {
  struct IssueNonDataOpcode : Event {
    explicit IssueNonDataOpcode(std::size_t ch, Opcode opcode)
      : ch_(ch), opcode_(opcode)
    {}
    bool execute(Driver* d) override {
      d->issue(ch_, opcode_);
      return d->ack(ch_);
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
    bool execute(Driver* d) override {
      d->issue(ch_, opcode_, dat_);
      return d->ack(ch_);
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
    bool execute(Driver*) override {
      return (--cycles_ != 0);
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
    bool execute(Driver* d) override {
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

bool StkTest::on_negedge_clk() {
  // TODO(stephenry): should be an assertion.
  if (event_queue_.empty()) return false;

  Driver* driver{kernel_->driver()};
  driver->idle();

  if (!event_queue_.front()->execute(driver)) {
    event_queue_.pop_front();
  }

  // If more events to go, reschedule, otherwise, try to reprogram
  // further stimulus, and then reschedule iff more arrives.
  //
  return !event_queue_.empty() || program();
}

bool StkTest::run() {
  program();
  const bool success = kernel_->run(this);
  return !success;
}

} // namespace tb_stk
