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

#ifndef QS_TB_RANDOM_H
#define QS_TB_RANDOM_H

#include <random>

namespace tb {

// Randomization support
//
struct Random {
  // Initialize random state
  static void init(unsigned seed);

  // Get current random state.
  static std::mt19937& mt() { return mt_; }


  // Generate a random integral type in range [lo, hi]
  template<typename T>
  static std::enable_if_t<std::is_integral_v<T>, T>
  uniform(T hi = std::numeric_limits<T>::max(),
          T lo = std::numeric_limits<T>::min()) {
    std::uniform_int_distribution<T> d(lo, hi);
    return d(mt_);
  }

  // Generate a random integral type in range [lo, hi]
  template<typename T>
  static std::enable_if_t<std::is_floating_point_v<T>, T>
  uniform(T hi = std::numeric_limits<T>::max(),
          T lo = std::numeric_limits<T>::min()) {
    std::uniform_real_distribution<T> d(lo, hi);
    return d(mt_);
  }

  template<typename T>
  static std::enable_if_t<std::is_floating_point_v<T>, T>
  normal(T mean, T stddev) {
    // Normal distribution from which to select price.
    std::normal_distribution<double> d(mean, stddev);
    return d(mt_);
  }

  // Generate a boolean with true probability 'true_prob'.
  static bool boolean(double true_prob = 0.5f);


  template<typename FwdIt>
  static FwdIt select_one(FwdIt begin, FwdIt end) {
    if (begin == end) return end;
    std::advance(begin, Random::uniform(std::distance(begin, end) - 1, 0l));
    return begin;
  }
  
  
 private:
  static inline std::mt19937 mt_{1};
};

} // namespace tb

#endif
