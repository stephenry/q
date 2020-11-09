//========================================================================== //
// Copyright (c) 2020, Stephen Henry
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
//========================================================================== //

#include "tb.h"
#include "vsupport.h"
#include "test.h"
#include "utility.h"
#include "vobj/Vtb_qs.h"
#ifdef OPT_TRACE_ENABLE
#  include <iostream>
#endif
#ifdef OPT_VCD_ENABLE
#  include "verilated_vcd_c.h"
#endif
#include "gtest/gtest.h"

namespace tb {

std::ostream& operator<<(std::ostream& os, const std::vector<word_type>& dp) {
  os << "[";
  for (std::size_t i = 0; i < dp.size(); i++) {
    if (i != 0) {
      os << ", ";
    }
    os << dp[i];
  }
  os << "]";
  return os;
}

// Set unsorted input interface.
void VSignals::set(const VIn& in) const {
  vsupport::set(in_vld_, in.vld);
  vsupport::set(in_sop_, in.sop);
  vsupport::set(in_eop_, in.eop);
  vsupport::set(in_dat_, in.dat);
}

void VSignals::set_clk(bool clk) {
  vsupport::set(clk_, clk);
}

void VSignals::set_rst(bool rst) {
  vsupport::set(rst_, rst);
}

// Sample 'in_rdy_r' port.
bool VSignals::in_rdy_r() const {
  return vsupport::get_as_bool(in_rdy_r_);
}

// Sample sorted output interface.
void VSignals::get(VOut& out) const {
  out.vld = vsupport::get(out_vld_r_);
  out.sop = vsupport::get(out_sop_r_);
  out.eop = vsupport::get(out_eop_r_);
  out.dat = vsupport::get(out_dat_r_);
}

// Get microcode instruction packet.
void VSignals::get(UCInst& inst) const {
  inst.commit = vsupport::get_as_bool(uc_inst_commit_);
  inst.inst = vsupport::get(uc_inst_);
}

  // Get microcode writeback packet
void VSignals::get(UCWriteback& wrbk) const {
  wrbk.wen = vsupport::get_as_bool(uc_rf_wen_);
  wrbk.wa = vsupport::get(uc_rf_wa_);
  wrbk.wdata = vsupport::get(uc_rf_wdata_);
}

// Obtain current simulation cycle.
vluint64_t VSignals::cycle() const {
  return vsupport::get(tb_cycle_);
}

class Instruction {
 public:
  Instruction(vluint16_t i)
      : i_(i)
  {}

  std::string dis() const {
    using std::to_string;
    
    std::string s;

    switch (opcode()) {
      case 0x0: {
        s += "nop";
      } break;
      case 0x1: {
        // Jcc
        s += "J";
        s += " ";
        switch (cc()) {
          case 1:
            s += "eq";
            break;
          case 2:
            s += "gt";
            break;
          case 3:
            s += "le";
            break;
        }
        s += " ";
        s += to_string(A());
      } break;
      case 0x2: {
        // Push/Pop:
        if (sel0()) {
          s += "pop";
          s += " ";
          s += reg(u());
        } else {
          s += "push";
          s += " ";
          s += reg(r());
        }
      } break;
      case 0x4: {
        // Ld/St:
        if (sel0()) {
          // St
          s += "st";
          s += " [";
          s += reg(r());
          s += "], ";
          s += reg(u());
        } else {
          // Ld
          s += "ld";
          s += " ";
          s += reg(r());
          s += ", [";
          s += reg(u());
          s += "]";
        }
      } break;
      case 0x6: {
        // Mov:
        s += "mov";
        if (!sel0() && !sel1()) {
          // Mov
          s += " ";
          s += reg(r());
          s += ", ";
          s += reg(u());
        } else if (!sel0() && sel1()) {
          // Movi
          s += "i";
          s += ", ";
          s += to_string(i());
        } else if(sel0()) {
          // Movs
          s += "s";
          s += " ";
          s += reg(r());
          s += ", ";
          s += regs(S());
        }
      } break;
      case 0x7: {
        // Add/Sub:
        if (sel0()) {
          // Sub
          s += "sub";
          s += sel1() ? "i " : " ";
          s += reg(r());
          s += sel1() ? to_string(i()) : reg(u());
        } else {
          // Add
          s += "add";
          s += sel1() ? "i " : " ";
          s += reg(r());
          s += sel1() ? to_string(i()) : reg(u());
        }
      } break;
      case 0xA: {
        // Call/Ret:
        if (sel0()) {
          s += "ret";
        } else {
          s += "call";
          s += " ";
          s += hex(A());
        }
      } break;
      case 0xF: {
        // Wait/Emit
        s += sel0() ? "emit" : "wait";
      }
    }

    return s;
  }

  vluint16_t opcode() const {
    return mask_bits(i_, 15, 12);
  }

  bool sel0() const {
    return get_bit(i_, 11);
  }

  bool sel1() const {
    return get_bit(i_, 3);
  }

  vluint8_t cc() const {
    return mask_bits(i_, 9, 8);
  }

  vluint16_t r() const {
    return mask_bits(i_, 10, 8);
  }

  vluint16_t s() const {
    return mask_bits(i_, 6, 4);
  }

  vluint16_t S() const {
    return mask_bits(i_, 3, 0);
  }

  vluint16_t i() const {
    return mask_bits(i_, 2, 0);
  }

  vluint16_t A() const {
    return mask_bits(i_, 7, 0);
  }

  vluint16_t u() const {
    return mask_bits(i_, 3, 0);
  }

 private:
  std::string reg(std::size_t i) const {
    using std::to_string;

    std::string s;
    s += "r";
    s += to_string(i);
    return s;
  }

  std::string regs(std::size_t i) const {
    using std::to_string;
    
    switch (i) {
      case 0: {
        return "N";
      } break;
      default: {
        ADD_FAILURE() << "Invalid special register id " << to_string(i) << "\n";
        return "Invalid";
      } break;
    }
  }
  
  vluint16_t i_;
};

Model::Model(TB* tb)
    : tb_(tb), is_valid_(true)
{}

void Model::step() {
  if (!is_valid_) return;
  
  VSignals::UCInst ucinst;
  tb_->vs_.get(ucinst);
  if (ucinst.commit) {
    const Instruction inst(ucinst.inst);
#ifdef OPT_TRACE_ENABLE
    std::cout << tb_->cycle() << " ("
              << tb::hex(ucinst.pc) << "): " << inst.dis();

    switch (inst.opcode()) {
      case 0: {
        // Nop
      } break;
      case 0x1: {
        // Jcc
      } break;
      case 0x2: {
        // Push/Pop:
      } break;
      case 0x4: {
        // Ld/St
      } break;
      case 0x6: {
        // Mov/Movi/Movs
      } break;
      case 0x7: {
        // Add/Sub
      } break;
      case 0xA: {
        // Call/Ret
      } break;
      case 0xF: {
        // Wait/Emit
      } break;
      default: {
        ADD_FAILURE() << "Invalid instruction decoded.";
      };
    }
    
    std::cout << "\n";
#endif
  }
}


TB::TB(const Options& opts)
    : opts_(opts) {
#ifdef OPT_VCD_ENABLE
  if (opts.wave_enable) {
    Verilated::traceEverOn(true);
  }
#endif
  u_ = new Vtb_qs;
  vs_ = VSignals::bind(u_);
#ifdef OPT_VCD_ENABLE
  if (opts.wave_enable) {
    wave_ = new VerilatedVcdC;
    u_->trace(wave_, 99);
    wave_->open(opts.wave_name.c_str());
#ifdef OPT_TRACE_ENABLE
    std::cout << "[TB] Dumping to VCD: " << opts.wave_name << "\n";
#endif
  }
#endif
  // Construct behavioral model
  model_ = Model(this);
}

TB::~TB() {
  delete u_;
#ifdef OPT_VCD_ENABLE
  if (wave_) {
    wave_->close();
    delete wave_;
  }
#endif
}

void TB::push_back(const std::vector<word_type>& pkt) {
  pkts_.push_back(pkt);
}

// Run simulation.
void TB::run() {
  cycle_ = 0;
  time_ = 0;

  // Run reset
  reset();

  while (!pkts_.empty()) {
    // Obtain stimulus packet
    const std::vector<word_type> data_packet{pkts_.front()};
    pkts_.pop_front();
#ifdef OPT_TRACE_ENABLE
    log() << "Attempting: " << data_packet << "\n";
#endif

    // Drive packet
    drive_stimulus(data_packet);
    
    // Compuare output
    const std::vector<word_type> actual /* = get_sorted_packet() */;
#ifdef OPT_TRACE_ENABLE
    log() << "Received: " << actual << "\n";
#endif
    const std::vector<word_type> expected = test::compute_expected(actual);
    EXPECT_EQ(actual, expected);
  }
  
  // Wind-down simulation
  step(20);
}
#ifdef OPT_TRACE_ENABLE

std::ostream& TB::log() const {
  std::ostream& os = std::cout;
  return os << "TB @" << cycle() << ": ";
}
#endif

void TB::reset() {
  vs_.set_rst(false);
  for (vluint64_t i = 0; i < 20; i++) {
    vs_.set_rst((i > 5) && (i < 15));
    step();
  }
  vs_.set_rst(false);
}


void TB::drive_stimulus(const std::vector<word_type>& stimulus) {
  bool done;

  VSignals::VIn in;
  for (std::size_t i = 0; i < stimulus.size(); i) {
    if (true || vs_.in_rdy_r()) {

      // Drive interface
      in.vld = true;
      in.sop = (i == 0);
      in.eop = (i == (stimulus.size() - 1));
      in.dat = stimulus[i];
      vs_.set(in);

      // Advance beat.
      i++;
    }

    step();
  }

  // Drive to idle.
  in.vld = false;
  vs_.set(in);
}

std::vector<word_type> TB::get_sorted_packet() {
  std::vector<word_type> r;

  VSignals::VOut out;
  bool done = false;
  bool got_sop = false;

  do {
    vs_.get(out);

    // If valid beat, otherwise ignore.
    if (out.vld) {

      // Check SOP:
      if (out.sop) {
        // Should not see SOP twice within the same packet.
        EXPECT_TRUE(r.empty());

        got_sop = true;
      }

      // Check EOP:
      if (out.eop) {
        // Should have seen SOP before EOP
        EXPECT_TRUE(got_sop);

        // Final word in packet, we're good.
        done = true;
      }

      // Sample data
      r.push_back(out.dat);

      // Advance model.
      step();
    }

  } while (!done);
  
  return r;
}

void TB::step(std::size_t n) {
  while (n-- > 0) {
    // !CLK region
    vs_.set_clk(false);
    u_->eval();
#ifdef OPT_VCD_ENABLE
    if (wave_) {
      wave_->dump(time());
    }
#endif
    model_.step();
    time_ += 5;

    // CLK region
    vs_.set_clk(true);
    ++cycle_;
    u_->eval();
#ifdef OPT_VCD_ENABLE
    if (wave_) {
      wave_->dump(time());
    }
#endif
    model_.step();
    time_ += 5;
  }
}

} // namespace tb
