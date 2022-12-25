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
//========================================================================== //

#ifndef V_TB_RND_H
#define V_TB_RND_H

#include <random>
#include <vector>
#include <algorithm>
#include "verilated.h"
#include "sim.h"

class Random {
  using seed_type = std::mt19937::result_type;

 public:
  explicit Random(seed_type s = seed_type{}) { seed(s); }

  // Set seed of randomization engine.
  void seed(seed_type s) { mt_.seed(s); }

  // Generate a random integral type in range [lo, hi]
  template <typename T>
  std::enable_if_t<std::is_integral_v<T>, T> uniform(
      T hi = std::numeric_limits<T>::max(),
      T lo = std::numeric_limits<T>::min()) {
    std::uniform_int_distribution<T> d(lo, hi);
    return d(mt_);
  }

  // Generate a random integral type in range [lo, hi]
  template <typename T>
  std::enable_if_t<std::is_floating_point_v<T>, T> uniform(
      T hi = std::numeric_limits<T>::max(),
      T lo = std::numeric_limits<T>::min()) {
    std::uniform_real_distribution<T> d(lo, hi);
    return d(mt_);
  }

  template<std::size_t T_Words>
  void uniform(VlWide<T_Words>& r) {
    std::generate_n(r.data(), T_Words, [&]() {
      return uniform<WData>();
    });
  }

 private:
  std::mt19937 mt_;
};

template <typename T>
class Bag {
 public:
  explicit Bag() = default;

  void push_back(const T& t, float weight = 0.0f) {
    ts_.push_back(std::make_pair(weight_total_, t));
    weight_total_ += weight;
  }

  T pick() const {
    if (ts_.empty()) return T{};

    const float sel = Globals::random->uniform(0.0f, weight_total_);
    auto it = std::lower_bound(ts_.begin(), ts_.end(), sel,
      [](const std::pair<float, T>& lhs, const std::pair<float, T>& rhs) {
        return lhs.first < rhs.first;
      });
    return it->second;
  }

 private:
  float weight_total_ = 0.0f;
  std::vector<std::pair<float, T> > ts_;
};

#endif
