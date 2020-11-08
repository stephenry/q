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
   // Bank Interface                                                          //
   //                                                                         //
   //======================================================================== //

   //
   , input qs_pkg::bank_state_t                   bnk_in
   //
   , output logic                                 bnk_out_vld_r
   , output qs_pkg::bank_state_t                  bnk_out_r
   , output qs_pkg::bank_id_t                     bnk_idx_r
   //
   , input qs_pkg::w_t                            deq_rd_data_r
   //
   , output logic                                 deq_rd_en_r
   , output qs_pkg::addr_t                        deq_rd_addr_r

   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

   , input                                        clk
   , input                                        rst
);
/*
  // ======================================================================== //
  //                                                                          //
  // Wires                                                                    //
  //                                                                          //
  // ======================================================================== //

  typedef struct packed {
    logic 	 busy;
    logic [1:0]  state;
  } dequeue_fsm_encoding_t;
  
  typedef enum   logic [2:0] {  DEQUEUE_FSM_IDLE   = 3'b000,
                                DEQUEUE_FSM_UNLOAD = 3'b101
                                } dequeue_fsm_t;
  //
  `LIBV_REG_EN(dequeue_fsm_encoding_t, dequeue_fsm);
  `LIBV_REG_EN(qs_pkg::bank_id_t, dequeue_bank_idx);
  `LIBV_REG_EN(addr_t, dequeue_idx);
  `LIBV_SPSRAM_SIGNALS(dequeue_, W, $clog2(N));
  //
  qs_pkg::bank_state_t                  dequeue_bank;
  logic                                 dequeue_bank_en;
  //
  typedef struct packed {
    logic                sop;
    logic                eop;
    logic                err;
    qs_pkg::bank_id_t    idx;
  } dequeue_t;
  //
  `LIBV_REG_RST(logic, dequeue_out_vld, 1'b0);
  `LIBV_REG_EN(dequeue_t, dequeue_out);
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
  always_comb begin : dequeue_fsm_PROC

    // Defaults:
    dequeue_fsm_en 	= 'b0;
    dequeue_fsm_w 	= dequeue_fsm_r;

    dequeue_bank_en 	= 'b0;
    dequeue_bank 	= bank_state_r [dequeue_bank_idx_r];

    dequeue_bank_idx_en = 'b0;
    dequeue_bank_idx_w 	= bank_id_inc(dequeue_bank_idx_r);

    dequeue_idx_en 	= 'b0;
    dequeue_idx_w 	= dequeue_idx_r + 'b1;

    // Dequeue out defaults.
    dequeue_out_vld_w 	= 'b0;
    dequeue_out_en 	= 'b0;
    dequeue_out_w 	= dequeue_out_r;

    // Bank defaults:
    dequeue_en 		= 'b0;
    dequeue_wen 	= '0;
    dequeue_addr 	= dequeue_idx_r;
    dequeue_din 	= '0;
    
    case (dequeue_fsm_r)

      DEQUEUE_FSM_IDLE: begin

	case (dequeue_bank.status)

	  BANK_SORTED: begin
	    // Update bank status.
	    dequeue_bank_en 	= 'b1;
	    dequeue_bank.status = BANK_UNLOADING;

	    // Reset index counter.
	    dequeue_idx_en 	= 'b1;
	    dequeue_idx_w 	= '0;

	    // Transition to 
	    dequeue_fsm_en 	= 'b1;
	    dequeue_fsm_w 	= DEQUEUE_FSM_UNLOAD;
	  end

	  default:
	    // Otherwise, continue to wait completion of the current
	    // bank.
	    ;

	endcase // case (dequeue_bank)

      end

      DEQUEUE_FSM_UNLOAD: begin

	// Load bank
	dequeue_en 	  = 'b1;

	dequeue_out_vld_w = 'b1;

	dequeue_out_en 	  = out_vld_w;
	dequeue_out_w 	  = '0;
	dequeue_out_w.sop = (dequeue_idx_r == '0);
	dequeue_out_w.idx = dequeue_bank_idx_r;

	if (dequeue_bank.n == dequeue_idx_r) begin
	  // Is final word, update status and return to IDLE.
	  dequeue_out_w.eop   = '1;
	  dequeue_out_w.err   = dequeue_bank.err;

	  dequeue_bank_en     = 'b1;
	  dequeue_bank.status = BANK_READY;

	  dequeue_fsm_en      = 'b1;
	  dequeue_fsm_w       = DEQUEUE_FSM_IDLE;
	end

      end // case: DEQUEUE_FSM_UNLOAD

      default:
	// Otherwise, invalid state
	;

    endcase // case (dequeue_fsm_r)

  end // block: dequeue_fsm_PROC

  // ------------------------------------------------------------------------ //
  //
  always_comb begin : out_PROC

    //
    out_vld_w = dequeue_out_vld_r;

    // Stage outputs
    out_en    = out_vld_w;
    out_w.sop = dequeue_out_r.sop;
    out_w.eop = dequeue_out_r.eop;
    out_w.err = dequeue_out_r.err;
    out_w.dat = bank_dout [dequeue_out_r.idx];

    // Drive outputs
    out_sop_r = out_r.sop;
    out_eop_r = out_r.eop;
    out_err_r = out_r.err;
    out_dat_r = out_r.dat;

  end // block: out_PROC
*/
endmodule // qs_deq
