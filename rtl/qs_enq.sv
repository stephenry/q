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

module qs_enq (

   //======================================================================== //
   //                                                                         //
   // Unsorted                                                                //
   //                                                                         //
   //======================================================================== //

   //
     input                                        in_vld
   , input                                        in_sop
   , input                                        in_eop
   , input [qs_pkg::W - 1:0]                      in_dat
   //
   , output logic                                 in_rdy

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
   , output logic                                 wr_en_r
   , output qs_pkg::addr_t                        wr_addr_r
   , output qs_pkg::w_t                           wr_data_r

   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

   //
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
    logic        ready;
    logic [1:0]  state;
  } enqueue_fsm_encoding_t;
  
  typedef enum   logic [3:0] {  ENQUEUE_FSM_IDLE = 4'b0000,
                                ENQUEUE_FSM_LOAD = 4'b1101
                                } enqueue_fsm_t;
  //
  `LIBV_REG_EN(enqueue_fsm_encoding_t, enqueue_fsm);
  `LIBV_REG_EN(bank_id_t, enqueue_bank_idx);
  `LIBV_REG_EN(addr_t, enqueue_idx);
  `LIBV_SPSRAM_SIGNALS(enqueue_, qs_pkg::W, $clog2(qs_pkg::N));
  //
  qs_pkg::bank_state_t                  enqueue_bank;
  logic                                 enqueue_bank_en;

  // ======================================================================== //
  //                                                                          //
  // Combinatorial Logic                                                      //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  always_comb begin : enqueue_PROC

    // Defaults:

    // FSM state
    enqueue_fsm_en 	= 'b0;
    enqueue_fsm_w 	= enqueue_fsm_r;

    // Bank state
    enqueue_bank 	= bank_state_r [enqueue_bank_idx_r];
    enqueue_bank_en 	= 'b0;

    // Bank index
    enqueue_bank_idx_en = 'b0;
    enqueue_bank_idx_w 	= bank_id_inc(enqueue_bank_idx_r);

    enqueue_idx_en 	= 'b0;
    enqueue_idx_w 	= enqueue_idx_r + 'b1;
    
    // Enqueue bank update
    enqueue_en 		= 'b0;
    enqueue_wen 	= 'b1;
    enqueue_addr 	= '0;
    enqueue_din 	= in_dat;

    case (enqueue_fsm_r)

      ENQUEUE_FSM_IDLE: begin
	// Enqueue FSM is IDLE awaiting for the current selected bank
	// to become READY. When it becomes READY, bank transitions to
	// the LOADING status and entries are pushed from the IN
	// interface.

	case (enqueue_bank.status)
	  BANK_IDLE: begin
	    // Update bank status
	    enqueue_bank_en 	 = 'b1;

	    enqueue_bank 	 = '0;
	    enqueue_bank.err 	 = 'b0;
	    enqueue_bank.n 	 = '0;
	    enqueue_bank.status  = qs_pkg::BANK_LOADING;

	    // Reset index
	    enqueue_idx_en 	 = 'b1;
	    enqueue_idx_w 	 = '0;

	    // Advance state.
	    enqueue_fsm_en = 'b1;
	    enqueue_fsm_w  = ENQUEUE_FSM_LOAD;
	  end
	  default: ;
	endcase // case (bank_state_r [enqueue_bank_idx_r].status)

      end // case: ENQUEUE_FSM_IDLE

      ENQUEUE_FSM_LOAD: begin
	// Enqueue FSM is loading data from the IN interface.

	casez ({in_vld, in_eop}) 
	  2'b1_0: begin
	    // Write to nominated bank.
	    enqueue_en 	   = 'b1;
	    enqueue_bank.n = enqueue_bank.n + 'b1;

	    // Advance index.
	    enqueue_idx_en = 'b1;
	  end
	  2'b1_1: begin
	    // Write to nominated bank.
	    enqueue_en 		= 'b1;
	    
	    // Update bank status, now ready to be sorted.
	    enqueue_bank_en 	= 'b1;
	    enqueue_bank.n 	= enqueue_idx_r;
	    enqueue_bank.status = qs_pkg::BANK_READY;

	    // Done, transition back to idle state.
	    enqueue_fsm_en 	= 'b1;
	    enqueue_fsm_w 	= ENQUEUE_FSM_IDLE;
	  end
	  default: begin
	    // Otherwise, bubble. Do nothing.
	  end
	endcase
	
      end // case: ENQUEUE_FSM_LOAD

      default:
	// Otherwise, invalid state.
	;

    endcase // case (enqueue_fsm_r)
    
    // Bank is ready to be loaded.
    in_rdy = enqueue_fsm_r.ready;

  end // block: enqueue_PROC
*/
endmodule // qs_enq
