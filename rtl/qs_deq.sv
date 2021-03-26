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

`timescale 1ns/1ps

`default_nettype none

`include "qs_pkg.vh"
`include "libv_pkg.vh"

module qs_deq (

   //======================================================================== //
   //                                                                         //
   // Sorted                                                                  //
   //                                                                         //
   //======================================================================== //

   //
     output logic                                 out_vld_r
   , output logic                                 out_sop_r
   , output logic                                 out_eop_r
   , output logic                                 out_err_r
   , output logic [qs_pkg::W - 1:0]               out_dat_r

   //======================================================================== //
   //                                                                         //
   // Bank Selection                                                          //
   //                                                                         //
   //======================================================================== //

   , output qs_pkg::bank_id_t                     bank_idx_r

   //======================================================================== //
   //                                                                         //
   // Scoreboard Interface                                                    //
   //                                                                         //
   //======================================================================== //

   //
   , input qs_pkg::bank_state_t                   bank_in
   //
   , output logic                                 bank_out_vld
   , output qs_pkg::bank_state_t                  bank_out

   //======================================================================== //
   //                                                                         //
   // Memory Bank Interface                                                   //
   //                                                                         //
   //======================================================================== //

   //
   , input                                        rd_data_vld_r
   , input qs_pkg::w_t                            rd_data_r
   //
   , output logic                                 rd_en_r
   , output qs_pkg::addr_t                        rd_addr_r

   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

   , input                                        clk
   , input                                        rst
);

  // ======================================================================== //
  //                                                                          //
  // Wires                                                                    //
  //                                                                          //
  // ======================================================================== //

  typedef struct packed {
    logic        busy;
    logic [1:0]  state;
  } fsm_encoding_t;

  typedef enum   logic [2:0] {  FSM_IDLE     = 3'b000,
                                FSM_UNLOAD   = 3'b101,
                                FSM_WAIT_EOP = 3'b110
                                } fsm_t;
  //
  `LIBV_REG_EN(fsm_encoding_t, fsm);
  `LIBV_REG_EN_W(qs_pkg::bank_id_t, bank_idx);
  `LIBV_REG_RST_W(logic, rd_en, 'b0);
  `LIBV_REG_EN_W(qs_pkg::addr_t, rd_addr);
  `LIBV_REG_EN(qs_pkg::addr_t, deq_ptr);
  //
  typedef struct packed {
    logic        sop;
    logic        eop;
    logic        err;
    qs_pkg::w_t  dat;
  } out_t;
  //
  `LIBV_REG_RST_W(logic, out_vld, 'b0);
  `LIBV_REG_EN(out_t, out);

  // ======================================================================== //
  //                                                                          //
  // Combinatorial Logic                                                      //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  always_comb begin : fsm_PROC

    // Defaults:
    fsm_en       = 'b0;
    fsm_w        = fsm_r;

    bank_out_vld = 'b0;
    bank_out     = bank_in;

    bank_idx_en  = 'b0;
    bank_idx_w   = qs_pkg::bank_id_inc(bank_idx_r);

    rd_addr_en   = 'b0;
    rd_addr_w    = rd_addr_r + 'b1;

    // Bank defaults:
    rd_en_w      = 'b0;
    rd_addr_en   = 'b0;

    case (fsm_r)

      FSM_IDLE: begin

        case (bank_in.status)

          qs_pkg::BANK_SORTED: begin
            // Update bank status.
            bank_out_vld    = 'b1;
            bank_out.status = qs_pkg::BANK_UNLOADING;

            // Reset index counter.
            rd_en_w         = 'b1;
            
            rd_addr_en      = 'b1;
            rd_addr_w       = '0;

            // Transition to UNLOAD state
            fsm_en          = 'b1;
            fsm_w           = FSM_UNLOAD;
          end

          default:
            // Otherwise, continue to wait completion of the current
            // bank.
            ;

        endcase // case (dequeue_bank)

      end

      FSM_UNLOAD: begin
        // Load bank
        bank_out_vld = 'b1;
        if (bank_in.n == rd_addr_r) begin
          fsm_en          = 'b1;
          fsm_w           = FSM_WAIT_EOP;
        end else begin
          // Still reading.
          
          // Emit read commit
          rd_en_w      = 'b1;
          rd_addr_en   = 'b1;
        end

      end // case: FSM_UNLOAD

      FSM_WAIT_EOP: begin
        if (out_vld_r & out_eop_r) begin
          // Is final word, update status and return to IDLE.
          bank_out_vld    = 'b1;
          bank_out.status = qs_pkg::BANK_IDLE;

          bank_idx_en     = 'b1;


          // Unload operation has completed, await for the final EOP
          // to be emitted until transitioning back to the IDLE sate.
          fsm_en          = 'b1;
          fsm_w           = FSM_IDLE;
        end
      end

      default:
        // Otherwise, invalid state
        ;

    endcase // case (fsm_r)

  end // block: fsm_PROC

  // ------------------------------------------------------------------------ //
  //
  always_comb begin : out_PROC

    // Defaults:
    deq_ptr_en = 'b0;
    deq_ptr_w  = deq_ptr_r + 'b1;

    //
    out_vld_w  = '0;

    out_en     = 'b0;
    out_w.sop  = (deq_ptr_r == '0);
    out_w.eop  = (deq_ptr_r == bank_in.n);
    out_w.err  = '0;
    out_w.dat  = rd_data_r;

    case (fsm_r)

      FSM_IDLE: begin

        // On tradition out of IDLE state, initialize read pointer.
        deq_ptr_en = fsm_en;
        deq_ptr_w  = '0;
      end

      default: begin
        if (rd_data_vld_r) begin
          // Emit returning data.
          out_vld_w  = 'b1;
          out_en     = 'b1;

          deq_ptr_en = 'b1;
        end
      end

    endcase // block: out_PROC

    // Drive outputs
    out_sop_r = out_r.sop;
    out_eop_r = out_r.eop;
    out_err_r = out_r.err;
    out_dat_r = out_r.dat;

  end // block: out_PROC

endmodule // qs_deq
