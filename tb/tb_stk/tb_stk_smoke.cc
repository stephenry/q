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

#include "tb_stk/tb_stk_smoke.h"
#include "rnd.h"

struct tb_stk::smoke::Test::Impl {

  enum class State {
    WaitForInitialization,
    IssueForChannelN,
    WindDown,
    Terminate
  };

  explicit Impl(StkTest* t)
    : t_(t), state_(State::WaitForInitialization)
  {}

  bool program() {
    switch (state_) {
    case State::WaitForInitialization: {
      // Wait until internal Stack initialization has completed;
      // probably unnecessary as RTL will back-pressure anyway.
      //
      t_->wait_until(StkTest::EventType::EndOfInitialization);
      ch_ = 0;
      state_ = State::IssueForChannelN;
    } break;
    case State::IssueForChannelN: {

      // Push data to channel 0,
      //
      push_random_to_ch(ch_);

      // Wait some number of cycles to allow command to be issued.  (On
      // an empty queue, admission stage scheduler ought to choose the
      // Push opcode queue over Pop, but for the purpose of the smoke
      // test, we don't want to rely upon this constraint).
      //
      t_->wait(10);

      // Pop data from channel 0; expect the same data back.
      //
      pop_from_ch(ch_);

      // Wait some cycles to allow the prior instructions to execute.
      //
      t_->wait(100);

      if (++ch_ == cfg::ENGS_N) {
        state_ = State::WindDown;
      }
    } break;
    case State::WindDown: {
      state_ = State::Terminate;
    } break;
    case State::Terminate: {
    } break;
    }

    return (state_ != State::Terminate);
  }

  // Issue Push command to channel 'ch'
  //
  void push_random_to_ch(std::size_t ch) {
    VlWide<4> dat;
    Globals::random->uniform(dat);
    t_->issue(ch, Opcode::Push, dat);
  }

  // Issue Pop command to channel 'ch'
  //
  void pop_from_ch(std::size_t ch) {
    t_->issue(ch, Opcode::Pop);
  }

private:
  StkTest* t_;
  State state_;
  std::size_t ch_;
};

namespace tb_stk::smoke {

Test::Test() {
  impl_ = std::make_unique<Test::Impl>(this);
}

Test::~Test() {}

bool Test::program() {
  return impl_->program();
}

} // namespace tb_stk::smoke
