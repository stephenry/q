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

#ifndef QS_TB_UTILITY_H
#define QS_TB_UTILITY_H

#include <sstream>

namespace tb {

// Helper to convert some 'T' into hexadecimal representation.
template<typename T>
std::string hex(const T & t) {
  std::stringstream ss;
  ss << "0x" << std::hex << static_cast<vluint64_t>(t);
  return ss.str();
}

// -------------------------------------------------------------------------- //
// Function to derive a bit-mask for some integral type T.
template <typename T>
constexpr T mask(std::size_t bits) {
  static_assert(sizeof(T) <= 8);
  if (bits == 64) return static_cast<T>(0xFFFFFFFFFFFFFFFF);
  return static_cast<T>((1ull << bits) - 1);
}

// Mask bits in 't' such that only [msb:lsb] can be set.
template<typename T>
T mask_bits(T t, std::size_t msb, std::size_t lsb) {
  return t & (mask<T>(msb - lsb + 1) << lsb);
}

template<typename T>
constexpr bool get_bit(T t, std::size_t bit) {
  return ((t >> bit) & 0x1) != 0;
}

} // namespace

#endif
