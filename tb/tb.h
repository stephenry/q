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

#ifndef Q_TB_TB_H
#define Q_TB_TB_H

#include "cfg.h"
#include "sim.h"
#include "verilated.h"
#ifdef ENABLE_VCD
#include "verilated_vcd_c.h"
#endif
#include <iostream>

struct VSupport {

  static bool logic(vluint8_t* v) { return (*v != 0); }

  static void logic(vluint8_t* v, bool b) { *v = b ? 1 : 0; }
};

struct KernelCallBack {
  virtual ~KernelCallBack() = default;

  virtual void idle() {}

  virtual bool on_negedge_clk() { return false; }

  virtual bool on_posedge_clk() { return false; }
};

template<typename T, typename Driver>
class Kernel {
public:
  explicit Kernel() {
    vctxt_ = std::make_unique<VerilatedContext>();
    vtb_ = std::make_unique<T>(vctxt_.get());
#ifdef ENABLE_VCD
    if (Globals::vcd_on) {
      vctxt_->traceEverOn(true);
      vcd_ = std::make_unique<VerilatedVcdC>();
      vtb_->trace(vcd_.get(), 99);
      vcd_->open(Globals::vcd_fn.c_str());
    }
#endif
  }

  // Observers
  T* vtb() const { return vtb_.get(); }
  std::uint64_t tb_time() const { return tb_time_; }
  std::uint64_t tb_cycle() const { return 0; }

  bool run(KernelCallBack& cb) {
    tb_time_ = 0;

    // Drive all interfaces to a quiescent state.
    Driver::clk(vtb(), false);
    Driver::arst_n(vtb(), true);
    cb.idle();

    enum class ResetState {
      PreReset, InReset, PostReset
    };
    ResetState reset_state{ResetState::PreReset};
    int rundown_n = 5;
    bool do_stepping = true;
    bool failed = false;
    while (do_stepping || --rundown_n > 0) {
      if (++tb_time_ % 5 == 0) {
        const bool edge = Driver::clk(vtb());
        switch (reset_state) {
        case ResetState::PreReset: {
          const bool start_reset = (rundown_n-- == 0);
          Driver::arst_n(vtb(), !start_reset);
          if (start_reset) {
            rundown_n = 5;
            reset_state = ResetState::InReset;
          }
        } break;
        case ResetState::InReset: {
          const bool reset_complete = (rundown_n-- == 0);
          Driver::arst_n(vtb(), reset_complete);
          if (reset_complete) {
            rundown_n = 5;
            reset_state = ResetState::PostReset;
          }
        } break;
        case ResetState::PostReset: {
          if (do_stepping) {
            do_stepping = eval_clock_edge(cb, edge);
          }
        } break;
        }
        Driver::clk(vtb(), !edge);
      }
      vtb()->eval();
#ifdef ENABLE_VCD
      if (vcd_) { vcd_->dump(tb_time_); }
#endif
    }
    fini();
    return failed;
  }

  bool eval_clock_edge(KernelCallBack& cb, bool edge) {
    bool do_stepping;
    if (edge) {
      do_stepping = cb.on_negedge_clk();
    } else {
      do_stepping = cb.on_posedge_clk();
    }
    return do_stepping;
  }

  void fini() {
    vtb()->final();
#ifdef ENABLE_VCD
    if (vcd_) {
      vcd_->close();
      vcd_.reset(nullptr);
    }
#endif
  }

private:
#ifdef ENABLE_VCD
  std::unique_ptr<VerilatedVcdC> vcd_;
#endif
  std::unique_ptr<VerilatedContext> vctxt_;
  std::unique_ptr<T> vtb_;
  std::uint64_t tb_time_;
};

#endif
