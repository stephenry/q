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

`ifndef QS_RTL_QS_PKG_VH
`define QS_RTL_QS_PKG_VH

package qs_pkg;

  localparam int N = 16;

  localparam int W = 32;

  localparam int BANKS_N = 4;

  // Word type:
  typedef logic [W - 1:0]               w_t;

  // Sort vector addresss:
  typedef logic [$clog2(N) - 1:0]       addr_t;

  // Type to represent the number of entries in a vector.
  typedef logic [$clog2(N):0]           n_t;

  // Type to represent the index of a bank.
  typedef logic [$clog2(BANKS_N) - 1:0] bank_id_t;

  function automatic bank_id_t bank_id_inc(bank_id_t id); begin
    logic [$clog2(BANKS_N):0] sum = id + 'b1;

    bank_id_inc = (sum == N[$clog2(BANKS_N):0]) ? '0 : bank_id_t'(sum);
  end endfunction

  // Bank status encoding:
  typedef enum logic [2:0] {// Bank is unused and is not assigned.
                            BANK_IDLE      = 3'b000,
                            // Bank is being written.
                            BANK_LOADING   = 3'b001,
                            // Bank is ready to be sorted.
                            BANK_READY     = 3'b010,
                            // Bank is being sorted.
                            BANK_SORTING   = 3'b011,
                            // Bank has been sorted.
                            BANK_SORTED    = 3'b100,
                            // Bank is being read.
                            BANK_UNLOADING = 3'b101
                            } bank_status_t;


  typedef struct packed {
    // Flag denoting whether an error has occurred during processing.
    logic                     err;
    // Current bank word count (inclusive of final entry).
    addr_t                    n;
    // Current bank status:
    bank_status_t             status;
  } bank_state_t;
    
endpackage // qs_pkg

`endif //  `ifndef QS_RTL_QS_PKG_VH
