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
#include "log.h"
#include "vsupport.h"
#include <exception>
#include <utility>
#include <array>
#include <vector>
#include <deque>
#include <optional>
#include <sstream>

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

bool Driver::is_empty(std::size_t ch) const {
  switch (ch) {
  case 0: return VSupport::logic(&tb_stk_->o_empty0_r);
  case 1: return VSupport::logic(&tb_stk_->o_empty1_r);
  case 2: return VSupport::logic(&tb_stk_->o_empty2_r);
  case 3: return VSupport::logic(&tb_stk_->o_empty3_r);
  }
  // Never reached.
  return false;
}

bool Driver::is_full() const {
  return VSupport::logic(&tb_stk_->o_full_r);
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

void Driver::eval() const {
  tb_stk_->eval();
}

namespace tb_stk {

StkTest::StkTest() {
  if (Globals::logger) {
    test_scope_ = Globals::logger->top()->create_child("test");
  }
}

StkTest::~StkTest() {}

class Model::Impl {

  // Expected Response Data Type
  struct Expected {
    std::optional<VlWide<4>> data;
    Status status;
  };

  struct ActiveInvalidation {
    vluint8_t engid;
    vlsint64_t n;
  };

  // End-to-End latency of a command to response (in cycles).
  static constexpr vluint64_t LATENCY_N = 4;

  // Total number of lines that can be retained by the STK block.
  static constexpr vluint64_t CAPACITY_N =
    (cfg::LINES_PER_BANK_N * cfg::BANKS_N);

public:
  explicit Impl(KernelVerilated<Vtb_stk, Driver>* k)
    : k_(k), scope_(nullptr)
  {}

  void scope(Scope* scope) {
    scope_ = scope;
  }

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
    const vluint64_t cycle = k_->tb_cycle();
    switch (opcode) {
    case Opcode::Push: {
      Expected e;
      // TODO(stephenry); qualify on collisions.
      e.status = (size_ == CAPACITY_N) ? Status::ErrFull : Status::Okay;
      error = (e.status != Status::Okay);
      if (e.status == Status::Okay) {
        stks_[engid].push_back(dat);
        ++size_;
      }
      expect_.push_back(std::make_pair(cycle + LATENCY_N, e));
    } break;
    case Opcode::Pop: {
      Expected e;
      e.status = stks_[engid].empty() ? Status::ErrEmpty : Status::Okay;
      if (e.status == Status::Okay) {
        e.data = stks_[engid].back();
        stks_[engid].pop_back();
        --size_;
      }
      error = (e.status != Status::Okay);
      expect_.push_back(std::make_pair(cycle + LATENCY_N, e));
    } break;
    case Opcode::Inv: {
      if (!actinv_) {
        Expected e;
        // Attempt to invalidate empty Stack results in an ErrEmpty,
        // event though arguably it is does an error case.
        e.status = stks_[engid].empty() ? Status::ErrEmpty : Status::Okay;
        error = (e.status != Status::Okay);
        ActiveInvalidation ai;
        ai.engid = engid;
        ai.n = (stks_[engid].size() - 1);
        actinv_ = ai;
        stks_[engid].clear();
      } else {
        // Else, one invalidation is present in the system at any time.
        // This invalidation must belong to the same engid. Additionally,
        // the total invalidation count must equal the number of expected
        // entries in the stack, minus the first popped entry.
        //
        if (actinv_->engid != engid) {
          // We're seeing an invalidation request for engid that should
          // not be active.
          if (scope_) {
            scope_->Error("Bad ENGID seen for active invalidation operation ",
                          " Expected: ", actinv_->engid,
                          " Actual: ", engid);
          }
          error = true;
        } else if (actinv_->n-- == 0) {
          // We're seeing more invalidation requests than we had
          // predicted; we shouldn't be invalidating at this point.
          if (scope_) {
            scope_->Error("Unexpected invalidation request for engid: ",
                          actinv_->engid);
          }
          error = true;
        } else if (actinv_->n == 0) {
          // This is the last invalidation; discard active
          // invalidation state.
          if (scope_) {
            scope_->Info("INV operation completes for ENGID ", actinv_->engid);
          }
          actinv_.reset();
        }
      }
    } break;
    case Opcode::Nop: {
      // TBD
    } break;
    }

    if (scope_) {
      std::ostringstream ss;
      RecordRenderer rr{ss, "is"};
      rr.add("opcode", opcode);
      rr.add("engid", AsDec{engid});
      if (opcode == Opcode::Push) {
        rr.add("data", dat);
      }
      rr.finalize();
      scope_->Info("Issue: ", ss.str());
    }

    return !error;
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
      if (scope_) {
        scope_->Error(
          "Unexpected status response.",
          " Expected: ", e.status, " Actual: ", status);
      }
      success = false;
    }

    if (e.data && !VSupport::eq(*e.data, data)) {
      // Invalid data received on a pop.
      if (scope_) {
        scope_->Error(
          "Unexpected data response.",
          " Expected: ", *e.data, " Actual: ", data);
      }
      success = false;
    }

    if (scope_ && success) {
      std::ostringstream ss;
      RecordRenderer rr{ss, "rs"};
      rr.add("status", status);
      rr.finalize();
      scope_->Info("Response: ", ss.str());
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
  Scope* scope_;
  std::optional<ActiveInvalidation> actinv_;
};

Model::Model(KernelVerilated<Vtb_stk, Driver>* k) {
  impl_ = std::make_unique<Impl>(k);
}

Model::~Model() {}

void Model::scope(Scope* scope) { return impl_->scope(scope); }

bool Model::on_negedge_clk() { return impl_->on_negedge_clk(); }

void StkTest::issue(std::size_t ch, Opcode opcode, bool is_blocking) {
  class IssueNonDataOpcode : public Event {
    enum State {
      IssueCommand,
      WaitForResponse,
    };
  public:
    explicit IssueNonDataOpcode(std::size_t ch, Opcode opcode, bool is_blocking)
      : ch_(ch), opcode_(opcode), is_blocking_(is_blocking),
        state_(State::IssueCommand)
    {}
    bool execute(Driver* d) override {
      bool discard = false;
      switch (state_) {
      case State::IssueCommand: {
        d->issue(ch_, opcode_);
        // See note for IssueEvent
        d->eval();
        if (d->ack(ch_)) {
          state_ = State::WaitForResponse;
          discard = !is_blocking_;
        }
      } break;
      case State::WaitForResponse: {
        // Discard oprnands, simply awaiting response to be received.
        Status status;
        VlWide<4> dat;
        discard = d->tb_sample_response(status, dat);
      } break;
      }
      return discard;
    }
  private:
    State state_;
    std::size_t ch_;
    Opcode opcode_;
    bool is_blocking_;
  };
  event_queue_.push_back(
    std::make_unique<IssueNonDataOpcode>(ch, opcode, is_blocking));
}

void StkTest::issue(std::size_t ch, Opcode opcode, const VlWide<4>& dat,
                    bool is_blocking) {
  class IssueEvent : public Event {
    enum State {
      IssueCommand,
      WaitForResponse,
    };
  public:
    explicit IssueEvent(
      std::size_t ch, Opcode opcode, const VlWide<4>& dat, bool is_blocking)
      : ch_(ch), opcode_(opcode), dat_(dat), is_blocking_(is_blocking),
        state_(State::IssueCommand)
    {}
    bool execute(Driver* d) override {
      bool discard = true;
      switch (state_) {
      case State::IssueCommand: {
        d->issue(ch_, opcode_, dat_);
        // The path between valid to ack is combinatorial. This violates
        // commonly accepted constraints seen in interfaces such as AXI.
        // This is intentional. Admission logic in the pipeline
        // conditionally admits incoming commands based upon their
        // opcode and the occpuancy of the destination command queue. As
        // the STK block does not exist as a separate block, the
        // comb. path is fine in this circumstances.
        d->eval();
        if (d->ack(ch_)) {
          state_ = State::WaitForResponse;
          discard = !is_blocking_;
        }
      } break;
      case State::WaitForResponse: {
        // Discard oprnands, simply awaiting response to be received.
        Status status;
        VlWide<4> dat;
        discard = d->tb_sample_response(status, dat);
      } break;
      }
      return discard;
    }
  private:
    State state_;
    std::size_t ch_;
    Opcode opcode_;
    VlWide<4> dat_;
    bool is_blocking_;
  };
  event_queue_.push_back(
    std::make_unique<IssueEvent>(ch, opcode, dat, is_blocking));
}

void StkTest::wait(std::size_t cycles) {
  struct WaitCyclesEvent : Event {
    explicit WaitCyclesEvent(std::size_t cycles)
      : cycles_(cycles)
    {}
    bool execute(Driver*) override {
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
    bool execute(Driver* d) override {
      switch (et_) {
      case EventType::EndOfInitialization: {
        // Wait until after 10th cycle before sampling busy to allow
        // time for the initialization process to have been kicked off.
        const bool in_initialization = (d->tb_cycle() < 10) || d->busy_r();
        return !in_initialization;
      } break;
      }
      return true;
    }
  private:
    EventType et_;
  };

  event_queue_.push_back(std::make_unique<WaitUntilEvent>(et));
}

void StkTest::check_stack_state(StateType st, std::size_t ch) {
  struct CheckStackStateEvent : Event {
    explicit CheckStackStateEvent(StateType st, std::size_t ch = 0)
      : st_(st), ch_(ch), test_scope_(nullptr)
    {}
    void test_scope(Scope* test_scope) { test_scope_ = test_scope; }
    bool execute(Driver* d) override {
      switch (st_) {
      case StateType::IsFull: {
        if (!d->is_full()) {
          test_scope_->Error(
            "Stack is non-full; expected to be full.");
        }
      } break;
      case StateType::IsNonEmpty: {
        if (d->is_empty(ch_)) {
          test_scope_->Error(
            "Stack ID ", ch_, " expected to be non-empty, but is empty.");
        }
      } break;
      case StateType::IsEmpty: {
        if (!d->is_empty(ch_)) {
          test_scope_->Error(
            "Stack ID ", ch_, " expected to be empty, but is non-empty.");
        }
      } break;
      }
      return true;
    }
  private:
    std::size_t ch_;
    StateType st_;
    Scope* test_scope_;
  };

  auto e = std::make_unique<CheckStackStateEvent>(st, ch);
  e->test_scope(test_scope_);
  event_queue_.push_back(std::move(e));
}

bool StkTest::on_negedge_clk() {
  if (event_queue_.empty()) return false;

  Driver* driver{kernel_->driver()};
  driver->idle();

  if (event_queue_.front()->execute(driver)) {
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
