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

// Obtain current simulation cycle.
vluint64_t VSignals::cycle() const {
  return vsupport::get(tb_cycle_);
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
    const std::vector<word_type> actual = get_sorted_packet();
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
    if (vs_.in_rdy_r()) {

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
    time_ += 5;
  }
}

} // namespace tb
