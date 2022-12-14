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

#ifndef Q_TB_STK_TB_STK_MAIN_H
#define Q_TB_STK_TB_STK_MAIN_H

#include "test.h"
#include "tb.h"
#include "tb_stk/tb_stk.h"
#include "tb_stk/cfg.h"
#include "verilated.h"
#include "tb_stk/Vobj/Vtb_stk.h"
#include <deque>

// Forwards:
class TestRegistry;
class Scope;
class Vtb_stk;

namespace tb_stk {

class Driver {
public:

  explicit Driver() = default;

  explicit Driver(Vtb_stk* uut);

  void clk(bool c);

  bool clk() const;

  vluint64_t tb_cycle() const;

  void arst_n(bool c);

  bool arst_n() const;

  bool busy_r() const;

  void idle();

  bool is_empty(std::size_t ch) const;

  bool is_full() const;

  void issue(std::size_t ch, Opcode opcode);

  void issue(std::size_t ch, Opcode opcode, VlWide<4>& dat);

  bool ack(std::size_t ch) const;

  bool tb_sample_issue(vluint8_t& engid, Opcode& opcode, VlWide<4>& dat);

  bool tb_sample_response(Status& status, VlWide<4>& dat);

  void eval() const;

private:
  Vtb_stk* tb_stk_;
};

class Model {
  class Impl;
public:
  explicit Model(KernelVerilated<Vtb_stk, Driver>* k);
  ~Model();

  void scope(Scope* scope);

  bool on_negedge_clk();

private:
  std::unique_ptr<Impl> impl_;
};

class StkTest : public Test {

  struct Event {
    explicit Event() = default;
    virtual ~Event() = default;

    virtual bool execute(Driver*) = 0;
  };

public:
  class Factory;
  class Builder;

  enum class EventType {
    EndOfInitialization,
  };

  enum class StateType {
    IsFull,
    IsEmpty,
    IsNonEmpty,
  };

  explicit StkTest();
  virtual ~StkTest();

  bool run() override;

  virtual bool program() = 0;

  void wait(std::size_t cycles = 1);

  void wait_until(EventType et);

  void check_stack_state(StateType st, std::size_t ch = 0);

  void issue(std::size_t ch, Opcode opcode, bool is_blocking = false);

  void issue(std::size_t ch, Opcode opcode, const VlWide<4>& v,
             bool is_blocking = false);

protected:
  std::unique_ptr<KernelVerilated<Vtb_stk, Driver> > kernel_;
  std::unique_ptr<Model> model_;

  bool on_negedge_clk() override;
  std::deque<std::unique_ptr<Event> > event_queue_;
  Scope*  test_scope_{nullptr};
};

} // namespace tb_stk

#endif
