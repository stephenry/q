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
#include <array>
#include <vector>
#include <deque>
#include <optional>

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
  VSupport::logic(&tb_stk_->i_cmd0_vld, false);
  VSupport::logic(&tb_stk_->i_cmd1_vld, false);
  VSupport::logic(&tb_stk_->i_cmd2_vld, false);
  VSupport::logic(&tb_stk_->i_cmd3_vld, false);
}

void Driver::issue(std::size_t ch, Opcode opcode) {
  auto to_underlying = [](Opcode opcode) {
    return static_cast<std::underlying_type_t<Opcode>>(opcode);
  };
  switch (ch) {
  case 0: {
    VSupport::logic(&tb_stk_->i_cmd0_vld, true);
    tb_stk_->i_cmd0_opcode = to_underlying(opcode);
  } break;
  case 1: {
    VSupport::logic(&tb_stk_->i_cmd1_vld, true);
    tb_stk_->i_cmd1_opcode = to_underlying(opcode);
  } break;
  case 2: {
    VSupport::logic(&tb_stk_->i_cmd2_vld, true);
    tb_stk_->i_cmd2_opcode = to_underlying(opcode);
  } break;
  case 3: {
    VSupport::logic(&tb_stk_->i_cmd3_vld, true);
    tb_stk_->i_cmd3_opcode = to_underlying(opcode);
  } break;
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

bool Driver::tb_sample_issue(vluint8_t& engid, Opcode& opcode, VlWide<4>& dat) {
  const bool did_issue = VSupport::logic(&tb_stk_->o_lk_vld_w);
  if (did_issue) {
    engid = tb_stk_->o_lk_engid_w;
    opcode = Opcode{tb_stk_->o_lk_opcode_w};
    dat = tb_stk_->o_lk_dat_w;
  }
  return did_issue;
}

bool Driver::tb_sample_response(Status& status, VlWide<4>& dat) {
  const bool has_response = VSupport::logic(&tb_stk_->o_rsp_vld);
  if (has_response) {
    status = Status{tb_stk_->o_rsp_status};
    dat = tb_stk_->o_rsp_dat;
  }
  return has_response;
}

namespace tb_stk {

StkTest::StkTest() {}

StkTest::~StkTest() {}

class Model::Impl {

  // Expected Response Data Type
  struct Expected {
    std::optional<VlWide<4>> data;
    Status status;
  };

  // End-to-End latency of a command to response (in cycles).
  static constexpr vluint64_t LATENCY_N = 4;

  // Total number of lines that can be retained by the STK block.
  static constexpr vluint64_t CAPACITY_N =
    (cfg::LINES_PER_BANK_N * cfg::BANKS_N);

public:
  explicit Impl(KernelVerilated<Vtb_stk, Driver>* k)
    : k_(k)
  {}

  bool on_negedge_clk() {
    bool good = true;

    if (!sample_response())
      good = false;

    if (!sample_issue())
      good = false;

    return good;
  }

private:
  bool sample_issue() {
    vluint8_t engid;
    Opcode opcode;
    VlWide<4> dat;

    if (!k_->driver()->tb_sample_issue(engid, opcode, dat))
      return true;

    bool error = false;
    Status status = Status::Okay;
    const vluint64_t cycle = k_->tb_cycle();
    switch (opcode) {
    case Opcode::Push: {
      Expected e;
      // TODO(stephenry); qualify on collisions.
      status = (size_ == CAPACITY_N) ? Status::ErrFull : Status::Okay;
      e.status = status;
      if (e.status == Status::Okay) {
        stks_[engid].push_back(dat);
        ++size_;
      }
      expect_.push_back(std::make_pair(cycle + LATENCY_N, e));
    } break;
    case Opcode::Pop: {
      Expected e;
      status = stks_[engid].empty() ? Status::ErrEmpty : Status::Okay;
      e.status = status;
      if (e.status == Status::Okay) {
        e.data = stks_[engid].back();
        stks_[engid].pop_back();
        --size_;
      }
      expect_.push_back(std::make_pair(cycle + LATENCY_N, e));
    } break;
    case Opcode::Inv: {
      // TBD
    } break;
    case Opcode::Nop: {
      // TBD
    } break;
    }

    return (status == Status::Okay);
  }

  bool sample_response() {
    Status status;
    VlWide<4> data;

    const bool got_response =
      k_->driver()->tb_sample_response(status, data);

    const bool expect_response =
      !expect_.empty() && (expect_.front().first == k_->tb_cycle());

    if (got_response != expect_response) {
      // Expected response mismatch.
      return false;
    }

    if (!got_response) {
      // No response received.
      return true;
    }

    const Expected& e{expect_.front().second};

    bool success = true;
    if (e.status != status) {
      // Invalid response received.
      std::cout << "ERROR STATUS\n";
      success = false;
    }

    if (e.data && !VSupport::eq(*e.data, data)) {
      // Invalid data received on a pop.
      std::cout << "ERROR DATA\n";
      success = false;
    }

    expect_.pop_front();

    // Otherwise, no mismatches, comparsion has succeeded.
    return success;
  }

  // Pointer to UUT kernel.
  KernelVerilated<Vtb_stk, Driver>* k_;
  std::array<std::vector<VlWide<4>>, cfg::ENGS_N> stks_;
  std::deque<std::pair<vluint64_t, Expected> > expect_;
  std::size_t size_;
};

Model::Model(KernelVerilated<Vtb_stk, Driver>* k) {
  impl_ = std::make_unique<Impl>(k);
}

Model::~Model() {}

bool Model::on_negedge_clk() { return impl_->on_negedge_clk(); }

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

  // Update Model (and error out if necessary)
  if (!model_->on_negedge_clk()) return false;

  // Otherise, If more events to go, reschedule, otherwise, try to
  // reprogram further stimulus, and then reschedule iff more arrives.
  //
  return !event_queue_.empty() || program();
}

bool StkTest::run() {
  program();
  const bool success = kernel_->run(this);
  return !success;
}

} // namespace tb_stk
