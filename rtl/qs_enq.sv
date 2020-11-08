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
   , output logic                                 in_rdy_r

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
   , input qs_pkg::bank_state_t                   bank_in_r
   //
   , output logic                                 bank_out_vld_r
   , output qs_pkg::bank_state_t                  bank_out_r

   //======================================================================== //
   //                                                                         //
   // Memory Bank Interface                                                   //
   //                                                                         //
   //======================================================================== //

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

  // ======================================================================== //
  //                                                                          //
  // Wires                                                                    //
  //                                                                          //
  // ======================================================================== //

  typedef struct packed {
    logic 	 busy;
    logic        ready;
    logic [1:0]  state;
  } fsm_encoding_t;
  
  typedef enum   logic [3:0] {  FSM_IDLE = 4'b0000,
                                FSM_LOAD = 4'b1101
                                } fsm_t;
  //
  `LIBV_REG_EN(fsm_encoding_t, fsm);
  `LIBV_REG_EN_W(qs_pkg::bank_id_t, bank_idx);
  `LIBV_REG_RST_W(logic, wr_en, 'b0);
  `LIBV_REG_EN_W(qs_pkg::addr_t, wr_addr);
  `LIBV_REG_EN_W(qs_pkg::w_t, wr_data);
  `LIBV_REG_RST_W(logic, bank_out_vld, 'b0);
  `LIBV_REG_EN_W(qs_pkg::bank_state_t, bank_out);

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
    fsm_en 	   = 'b0;
    fsm_w 	   = fsm_r;

    // Bank state
    bank_out_vld_w = 'b0;
    bank_out_w 	   = bank_out_r;

    // Bank index
    bank_idx_en    = 'b0;
    bank_idx_w 	   = qs_pkg::bank_id_inc(bank_idx_r);

    // Memory bank defaults:
    wr_en_w 	   = 'b0;
    wr_addr_en 	   = 'b0;
    wr_addr_w 	   = wr_addr_r + 'b1;
    wr_data_en 	   = 'b0;
    wr_data_w 	   = in_dat;

    case (fsm_r)

      FSM_IDLE: begin
	// Enqueue FSM is IDLE awaiting for the current selected bank
	// to become READY. When it becomes READY, bank transitions to
	// the LOADING status and entries are pushed from the IN
	// interface.

	case (bank_in_r.status)
	  qs_pkg::BANK_IDLE: begin
	    // Update bank status
	    bank_out_vld_w 	= 'b1;

	    bank_out_w 	= '0;
	    bank_out_w.err 	= 'b0;
	    bank_out_w.n 	= '0;
	    bank_out_w.status = qs_pkg::BANK_LOADING;

	    // Reset index
	    wr_addr_en 		= 'b1;
	    wr_addr_w 		= '0;

	    // Advance state.
	    fsm_en 		= 'b1;
	    fsm_w 		= FSM_LOAD;
	  end
	  default: ;
	endcase // case (bank_in_r.status)

      end // case: FSM_IDLE

      FSM_LOAD: begin
	// Enqueue FSM is loading data from the IN interface.

	casez ({in_vld, in_eop})
	  2'b1_0: begin
	    // Write to nominated bank.
	    wr_en_w 	   = 'b1;
	    wr_data_en 	   = 'b1;
	    // Advance index.
	    wr_addr_en 	   = 'b1;
	  end
	  2'b1_1: begin
	    // Write to nominated bank.
	    wr_en_w 	      = 'b1;
	    wr_data_en 	      = 'b1;
	    // Advance index.
	    wr_addr_en 	      = 'b1;

	    // Write to nominated bank.
	    bank_out_vld_w    = 'b1;
	    
	    // Update bank status, now ready to be sorted.
	    bank_out_vld_w    = 'b1;
	    bank_out_w.n      = wr_addr_r;
	    bank_out_w.status = qs_pkg::BANK_READY;

	    // Done, transition back to idle state.
	    fsm_en 	      = 'b1;
	    fsm_w 	      = FSM_IDLE;
	  end
	  default: begin
	    // Otherwise, bubble. Do nothing.
	  end
	endcase
	
      end // case: FSM_LOAD

      default:
	// Otherwise, invalid state.
	;

    endcase // case (fsm_r)
    
    // Bank is ready to be loaded.
    in_rdy_r = fsm_r.ready;

  end // block: enqueue_PROC

endmodule // qs_enq
