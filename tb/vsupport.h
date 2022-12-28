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

#ifndef Q_TB_VSUPPORT_H
#define Q_TB_VSUPPORT_H

#include "log.h"
#include "verilated.h"
#include <algorithm>

struct VSupport {

  static bool logic(vluint8_t* v);

  static void logic(vluint8_t* v, bool b);

  template<std::size_t T_Words>
  static void zero(VlWide<T_Words>& d) {
    std::fill_n(d.data(), T_Words, 0);
  }

  template<std::size_t T_Size>
  static bool eq(const VlWide<T_Size>& lhs, const VlWide<T_Size>& rhs) {
    return std::equal(lhs.data(), lhs.data() + T_Size, rhs.data());
  }

};

template<std::size_t T_Size>
struct StreamRenderer<VlWide<T_Size>> {
  static void write(std::ostream& os, const VlWide<T_Size>& v) {
    const WData* d = v.data();
    for (std::size_t i = 0; i < T_Size; i++) {
      const bool showbase = (i == 0);
      StreamRenderer::write(os, AsHex{d[i], showbase});
    }
  }
};

#endif
