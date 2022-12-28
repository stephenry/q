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

#include "cfg.h"
#include "sim.h"
#include "test.h"
#include "rnd.h"
#include "log.h"
#include "tb_stk/tb_stk.h"
#include <iostream>
#include <vector>
#include <string_view>

namespace {

template <typename... Ts>
bool is_one_of(std::string_view in, Ts&&... ts) {
  for (std::string_view opt : {ts...}) {
    if (in == opt) return true;
  }
  return false;
}

class Driver {
public:
  explicit Driver(int argc, const char** argv);

  int execute();

private:
  void init_environment();
  void parse_args(int argc, const char** argv);
  void print_usage(std::ostream& os) const;

  TestRegistry tr_;
  Random rnd_;
  std::unique_ptr<Logger> logger_;
};

Driver::Driver(int argc, const char** argv) {
  init_environment();
  parse_args(argc, argv);
}

void Driver::init_environment() {
  tb_stk::init(tr_);
  Globals::random = std::addressof(rnd_);
}

void Driver::parse_args(int argc, const char** argv) {
  std::vector<std::string_view> vs{argv, argv + argc};
  for (int i = 1; i < vs.size(); ++i) {
    const std::string_view argstr{vs.at(i)};
    if (is_one_of(argstr, "-h", "--help")) {
      print_usage(std::cout);
      std::exit(1);
    } else if (is_one_of(argstr, "-v", "--verbose")) {
      logger_ = std::make_unique<Logger>();
      Globals::logger = logger_.get();
    } else if (is_one_of(argstr, "-s", "--seed")) {
      const std::string sstr{vs.at(++i)};
      Globals::random->seed(std::stoi(sstr));
    } else if (is_one_of(argstr, "--vcd")) {
      // --vcd: emit VCD of simulation.
#ifdef ENABLE_VCD
      Globals::vcd_on = true;
#else
      // VCD support has not been compiled into driver. Fail
      std::cout
          << "Waveform tracing has not been enabled in current build.\n";
#endif
    } else if (is_one_of(argstr, "-t", "--test")) {
      Globals::test_name = vs.at(++i);
      Globals::test_builder = tr_.get(Globals::test_name);
      if (!Globals::test_builder) {
        std::cout << "Test does not exist " << Globals::test_name << "\n";
        std::exit(1);
      }
    } else {
      std::cout << "Unknown argument: " << argstr << "\n";
    }
  }
}

int Driver::execute() {
  if (Globals::test_builder) {
    std::unique_ptr<Test> test{Globals::test_builder->construct()};
    test->run();
    return 0;
  }
  return 1;
}
void Driver::print_usage(std::ostream& os) const {
  os << "Usage is:\n"
     << "   -h|--help         Print help and quit.\n"
     << "   -v|--verbose      Verbose logging.\n"
     << "   -t|--test         Select testcase.\n"
     << "   -s|--seed         Randomization seed.\n"
     << "      --vcd          Trace VCD waveform.\n"
    ;
}

} // namespace

int main(int argc, const char** argv) {
  return Driver{argc, argv}.execute();
}
