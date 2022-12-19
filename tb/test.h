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

#ifndef Q_TB_TEST_H
#define Q_TB_TEST_H

#include <map>
#include <string>
#include <memory>

class Test {
public:
  explicit Test() = default;
  virtual ~Test() = default;

  virtual bool run() = 0;
};

class TestFactory {
public:
  explicit TestFactory() = default;
  virtual ~TestFactory() = default;

  virtual std::unique_ptr<Test> construct() = 0;
};

class TestRegistry {
public:
  explicit TestRegistry() = default;

  TestFactory* get(const std::string& name);

  template<typename T, typename ...Args>
  void add(const std::string& name, Args&& ...args) {
    tfm_.insert(std::make_pair(
      name, std::make_unique<T>(std::forward<Args>(args)...)));
  }

private:
  std::map<std::string, std::unique_ptr<TestFactory>> tfm_;
};

#endif
