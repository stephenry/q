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
#  include <sstream>
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

bool VSignals::get_rst() const {
  return vsupport::get_as_bool(rst_);
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
  inst.pc = vsupport::get(uc_inst_pc_);
}

  // Get microcode writeback packet
void VSignals::get(UCWriteback& wrbk) const {
  wrbk.wen = vsupport::get_as_bool(uc_rf_wen_);
  wrbk.wa = vsupport::get(uc_rf_wa_);
  wrbk.wdata = vsupport::get(uc_rf_wdata_);
  wrbk.flags_en = vsupport::get_as_bool(uc_flags_en_);
  wrbk.c = vsupport::get_as_bool(uc_flags_c_);
  wrbk.n = vsupport::get_as_bool(uc_flags_n_);
  wrbk.z = vsupport::get_as_bool(uc_flags_z_);
}

// Obtain current simulation cycle.
vluint64_t VSignals::cycle() const {
  return vsupport::get(tb_cycle_);
}

class Instruction {
  
 public:
  Instruction(vluint16_t enc)
      : enc_(enc)
  {}

  vluint16_t enc() const { return enc_; }

  std::string dis() const {
    using std::to_string;
    
    std::string s;

    switch (opcode_field()) {
      case 0x0: {
        s += "nop";
      } break;
      case 0x1: {
        // Jcc
        s += "j";
        switch (cc_field()) {
          case 1: {
            s += "eq";
          } break;
          case 2: {
            s += "gt";
          } break;
          case 3: {
            s += "le";
          } break;
        }
        s += " ";
        s += hex(A_field());
      } break;
      case 0x2: {
        // Push/Pop:
        if (sel0()) {
          s += "pop";
          s += " ";
          s += reg(r_field());
        } else {
          s += "push";
          s += " ";
          s += reg(u_field());
        }
      } break;
      case 0x4: {
        // Ld/St:
        if (sel0()) {
          // St
          s += "st";
          s += " [";
          s += reg(r_field());
          s += "], ";
          s += reg(u_field());
        } else {
          // Ld
          s += "ld";
          s += " ";
          s += reg(r_field());
          s += ", [";
          s += reg(u_field());
          s += "]";
        }
      } break;
      case 0x6: {
        // Mov:
        s += "mov";
        if (!sel0() && !sel1()) {
          // Mov
          s += " ";
          s += reg(r_field());
          s += ", ";
          s += reg(u_field());
        } else if (!sel0() && sel1()) {
          // Movi
          s += "i";
          s += " ";
          s += reg(r_field());
          s += ", ";
          s += to_string(i_field());
        } else if(sel0()) {
          // Movs
          s += "s";
          s += " ";
          s += reg(r_field());
          s += ", ";
          s += regs(S_field());
        }
      } break;
      case 0x7: {
        // Add/Sub:
        if (sel0()) {
          // Sub
          s += "sub";
          s += sel1() ? "i " : " ";
          s += reg(r_field());
          s += ", ";
          s += reg(s_field());
          s += ", ";
          s += sel1() ? to_string(i_field()) : reg(u_field());
        } else {
          // Add
          s += "add";
          s += sel1() ? "i " : " ";
          s += reg(r_field());
          s += ", ";
          s += reg(s_field());
          s += ", ";
          s += sel1() ? to_string(i_field()) : reg(u_field());
        }
      } break;
      case 0xC: {
        // Call/Ret:
        if (sel0()) {
          s += "ret";
        } else {
          s += "call";
          s += " ";
          s += hex(A_field());
        }
      } break;
      case 0xF: {
        // Await/Emit
        s += sel0() ? "emit" : "await";
      }
    }

    return s;
  }

  vluint16_t opcode_field() const {
    return extract_field(enc_, 15, 12);
  }

  bool sel0() const {
    return get_bit(enc_, 11);
  }

  bool sel1() const {
    return get_bit(enc_, 3);
  }

  vluint8_t cc_field() const {
    return extract_field(enc_, 9, 8);
  }

  vluint16_t r_field() const {
    return extract_field(enc_, 10, 8);
  }

  vluint16_t s_field() const {
    return extract_field(enc_, 6, 4);
  }

  vluint16_t S_field() const {
    return extract_field(enc_, 3, 0);
  }

  vluint16_t i_field() const {
    return extract_field(enc_, 2, 0);
  }

  vluint16_t A_field() const {
    return extract_field(enc_, 7, 0);
  }

  vluint16_t u_field() const {
    return extract_field(enc_, 3, 0);
  }


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
 private:  
  vluint16_t enc_;
};

Model::Model(TB* tb)
    : tb_(tb), is_valid_(true) {
  reset();
}

void Model::reset() {
  // Point PC back to reset vector.
  arch_.pc = 0;
  // Clear general purpose registers.
  for (std::size_t i = 0; i < 8; i++)
    arch_.rf[i] = 0;
  // Clear special registers
  arch_.N = 0;
  // Clear stack.
  arch_.stack.clear();
}

void Model::step() {
  if (!is_valid_) return;
  
  VSignals::UCInst ucinst;
  tb_->vs_.get(ucinst);
  if (ucinst.commit) {
    VSignals::UCWriteback ucwrbk;
    tb_->vs_.get(ucwrbk);
    const Instruction inst(ucinst.inst);
#ifdef OPT_TRACE_ENABLE
    std::stringstream ss;
    ss.seekp(0, std::ios::end);
    ss << tb_->cycle() << " [" << tb::hex(ucinst.pc) << "]: " << inst.dis();
       

    if (ucwrbk.wen) {
      std::stringstream::pos_type offset = ss.tellp();
      ss << std::string(50 - offset, ' ')
         << inst.reg(ucwrbk.wa) << " <- " << hex(ucwrbk.wdata)
         << "\t["
         << (ucwrbk.flags_en && ucwrbk.c ? "c" : " ")
         << (ucwrbk.flags_en && ucwrbk.n ? "n" : " ")
         << (ucwrbk.flags_en && ucwrbk.z ? "z" : " ")
         << "]";
    }
    ss << "\n";
    std::cout << ss.str();
#endif

    // Check expected program counter.
    EXPECT_EQ(arch_.pc, ucinst.pc);

    switch (inst.opcode_field()) {
      case 0: {
        // Nop

        // Advance to next instruction
        arch_.pc++;
      } break;
      case 0x1: {
        // Jcc
        bool is_taken = false;
        switch (inst.cc_field()) {
          case 0: {
            // Unconditional
            is_taken = true;
          } break;
          case 1: {
            // EQ condition
          } break;
          case 2: {
            // GT condition
          } break;
          case 3: {
            // LE condition
          } break;
        }
        // Update program counter.
        arch_.pc = is_taken ? inst.A_field() : (arch_.pc + 1);
      } break;
      case 0x2: {
        // Push/Pop:
        if (!inst.sel0()) {
          // Push
          const vluint8_t ra_expected = inst.u_field();
          arch_.stack.push_back(arch_.rf[ra_expected]);
        } else {
          // Pop

          // Should not pop from empty stack.
          EXPECT_FALSE(arch_.stack.empty());

          // Validate writeback destination register.
          const vluint8_t  wa_expected = inst.r_field();
          const vluint8_t  wa_actual = ucwrbk.wa;
          EXPECT_EQ(wa_expected, wa_actual);

          // Validate state written back.
          const vluint32_t wdata_expected = arch_.stack.back();
          const vluint32_t wdata_actual = ucwrbk.wdata;
          EXPECT_EQ(wdata_expected, wdata_actual);
        }

        // Advance to next instruction
        arch_.pc++;
      } break;
      case 0x4: {
        // Ld/St
        if (!inst.sel0()) {
          // Load

          // Expect a write to the register file to take place.
          EXPECT_TRUE(ucwrbk.wen);

          // Expect write registers to be equal
          const vluint8_t wa_expected = inst.r_field();
          const vluint8_t wa_actual = ucwrbk.wa;
          EXPECT_EQ(wa_expected, wa_actual);

          // Expect write back data to be correct.
          const vluint8_t ra_expected = inst.u_field();
          const vluint32_t wdata_expected = arch_.mem[ra_expected];
          const vluint32_t wdata_actual = ucwrbk.wdata;
          EXPECT_EQ(wdata_expected, wdata_actual);

          // Update architectural state.
          arch_.rf[wa_actual] = wdata_actual;
        } else {
          // Store

          const vluint8_t wa_expected = inst.s_field();
          const vluint8_t ra_expected = inst.u_field();
          const vluint32_t wdata_expected = arch_.rf[ra_expected];

          // Update architectural state.
          arch_.rf[wa_expected] = wdata_expected;
        }
        // Advance to next instruction
        arch_.pc++;
      } break;
      case 0x6: {
        // Mov/Movi/Movs

        // Expect write to register file.
        EXPECT_TRUE(ucwrbk.wen);

        // Expect write to correct register.
        const vluint8_t wa_expected = inst.r_field();
        const vluint8_t wa_actual = ucwrbk.wa;
        EXPECT_EQ(wa_expected, wa_actual);

        vluint32_t wdata_expected;

        if (       !inst.sel0() && !inst.sel1()) {
          // Mov
          const vluint8_t ra_expected = inst.u_field();
          wdata_expected = arch_.rf[ra_expected];
        } else if (!inst.sel0() &&  inst.sel1()) {
          // Movi
          wdata_expected = inst.i_field();
        } else if ( inst.sel0() ) {
          // Movs
          switch (inst.S_field()) {
            case 0: {
              // Writing 'N' register
              wdata_expected = arch_.N;
            } break;
          }
        } else {
          // Illegal encoding; unreachable.
          ADD_FAILURE() << "Bad MOV{i,s} instruction encoding.\n";
        }

        // Validate expected vs. actual
        const vluint32_t wdata_actual = ucwrbk.wdata;
        EXPECT_EQ(wdata_expected, wdata_actual);

        // Update architectural state
        arch_.rf[wa_expected] = wdata_actual;

        // Advance to next instruction
        arch_.pc++;
      } break;
      case 0x7: {
        // Add/Sub

        // Advance to next instruction
        arch_.pc++;
      } break;
      case 0xC: {
        // Call/Ret
        if (!inst.sel0()) {
          // Call
          arch_.pc = inst.A_field();
        } else {
          // Ret
          arch_.pc = arch_.rf[7];
        }

      } break;
      case 0xF: {
        // Await/Emit

        // Advance to next instruction
        arch_.pc++;
      } break;
      default: {
        ADD_FAILURE() << "Invalid instruction decoded; instruction was: "
                      << hex(inst.enc())
                      << "\n";
      };
    }
  }
}

void Model::set_special_register(vluint8_t i, vluint32_t v) {
  switch (i) {
    case 0: {
      arch_.N = v;
    } break;
    default: {
      ADD_FAILURE() << "Invalid special register index " << i;
    } break;
  }
}

void Model::set_memory_dims(std::size_t lo, std::size_t hi) {
  // Disregard lo
  arch_.mem.resize(hi + 1);
}

void Model::set_memory(std::size_t i, vluint32_t word) {
  arch_.mem[i] = word;
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

  // Construct behavioral model
  model_ = Model(this);

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
    // Fix up context set outside of the microprogram.
    model_.set_special_register(0, data_packet.size() - 1);
    model_.set_memory_dims(0, data_packet.size() - 1);
    for (std::size_t i = 0; i < data_packet.size(); i++) {
      model_.set_memory(i, data_packet[i]);
    }
    
    // Compuare output
    const std::vector<word_type> actual /* = get_sorted_packet() */;
#ifdef OPT_TRACE_ENABLE
    log() << "Received: " << actual << "\n";
#endif
    const std::vector<word_type> expected = test::compute_expected(actual);
    EXPECT_EQ(actual, expected);
  }
  
  // Wind-down simulation
  step(200);
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
    step(1);
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
    if (vs_.get_rst()) {
      model_.reset();
    } else {
      model_.step();
    }
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
