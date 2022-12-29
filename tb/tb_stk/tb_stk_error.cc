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

#include "tb_stk/tb_stk_error.h"
#include "rnd.h"

struct tb_stk::error::Test::Impl {

  enum class State {
    WaitForInitialization,
    CheckStacksAreEmpty,
    PushSomeData,
    PopMoreData,
    PushAllTheData,
    CheckIsFull,
    InvalidateEverything,
    TestComplete,
  };

  static constexpr std::size_t N = 2;

  static constexpr std::size_t LINES_TOTAL_N =
    (cfg::LINES_PER_BANK_N * cfg::BANKS_N);

  explicit Impl(StkTest* t)
    : t_(t), state_(State::WaitForInitialization)
  {}

  bool program() {
    bool done = false;
    switch (state_) {
    case State::WaitForInitialization: {
      // Wait until internal Stack initialization has completed;
      // probably unnecessary as RTL will back-pressure anyway.
      //
      t_->wait_until(StkTest::EventType::EndOfInitialization);
      state_ = State::CheckStacksAreEmpty;
    } break;
    case State::CheckStacksAreEmpty: {
      // Check test pre-conditions.
      for (std::size_t ch = 0; ch < cfg::ENGS_N; ch++) {
        t_->check_stack_state(StateType::IsEmpty, ch);
      }
      state_ = State::PushSomeData;
    } break;
    case State::PushSomeData: {
      for (std::size_t i = 0; i < N; i++) {
        VlWide<4> dat;
        Globals::random->uniform(dat);
        t_->issue(0, Opcode::Push, dat, true);
      }
      state_ = State::PopMoreData;
    } break;
    case State::PopMoreData: {
      // Pop more data than we expect to be in the stack; in this
      // case, the excess commands are converted to NOP and error out.
      for (std::size_t i = 0; i < (N + 2); i++) {
        t_->issue(0, Opcode::Pop, true);
      }
      n_ = 0;
      state_ = State::PushAllTheData;
    } break;
    case State::PushAllTheData: {
      VlWide<4> dat;
      // Fill Stack 0 to capacity...
      for (std::size_t i = 0; i < 256; i++) {
        Globals::random->uniform(dat);
        t_->issue(0, Opcode::Push, dat, true);
      }
      n_ += 256;
      if (n_ > LINES_TOTAL_N) {
        // ...and then, a wee bit more.
        state_ = State::CheckIsFull;
      }
    } break;
    case State::CheckIsFull: {
      t_->check_stack_state(StateType::IsFull);
      state_ = State::InvalidateEverything;
    } break;
    case State::InvalidateEverything: {
      t_->issue(0, Opcode::Inv, true);
      t_->wait(10);
      state_ = State::TestComplete;
    } break;
    case State::TestComplete: {
      done = true;
    } break;
    }
    return !done;
  }

private:
  StkTest* t_;
  State state_;
  std::size_t n_;
};

namespace tb_stk::error {

Test::Test() {
  impl_ = std::make_unique<Test::Impl>(this);
}

Test::~Test() {}

bool Test::program() {
  return impl_->program();
}

} // namespace tb_stk::error
