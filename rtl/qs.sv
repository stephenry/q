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

`include "qs_insts_pkg.vh"
`include "libv_pkg.vh"

module qs #(// The maximum number of entries in the sort vector.,
	    parameter int N = 16,
	    // The width of each element in the sort vector.
	    parameter int W = 32,
	    // Number of internal banks in which to queue/dequeue pending
	    // unsorted/sorted data
	    parameter int BANKS_N = 2) (

   //======================================================================== //
   //                                                                         //
   // Unsorted                                                                //
   //                                                                         //
   //======================================================================== //

   //
     input                                             in_vld
   , input                                             in_sop
   , input                                             in_eop
   , input [W - 1:0]                                   in_dat
   //
   , output logic                                      in_rdy

   //======================================================================== //
   //                                                                         //
   // Sorted                                                                  //
   //                                                                         //
   //======================================================================== //

   //
   , output logic                                      out_vld_r
   , output logic                                      out_sop_r
   , output logic                                      out_eop_r
   , output logic                                      out_err_r
   , output logic [W - 1:0]                            out_dat_r

   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

   , input                                        clk
   , input                                        rst
);

  // Local types:

  // Word type:
  typedef logic [W - 1:0] 		w_t;

  // Sort vector addresss:
  typedef logic [$clog2(N) - 1:0] 	addr_t;

  // Type to represent the number of entries in a vector.
  typedef logic [$clog2(N):0] 	        n_t;

  // Type to represent the index of a bank.
  typedef logic [$clog2(BANKS_N) - 1:0] bank_id_t;

  // Bank status encoding:
  typedef enum logic [2:0] {// Bank is unused and is not assigned.
			    BANK_IDLE 	   = 3'b000,
			    // Bank is being written.
                            BANK_LOADING   = 3'b001,
			    // Bank is ready to be sorted.
                            BANK_READY 	   = 3'b010,
			    // Bank is being sorted.
                            BANK_SORTING   = 3'b011,
			    // Bank has been sorted.
                            BANK_SORTED    = 3'b100,
			    // Bank is being read.
                            BANK_UNLOADING = 3'b101
                            } bank_status_t;


  typedef struct packed {
    // Flag denoting whether an error has occurred during processing.
    logic                     error;
    // Current bank word count.
    n_t                       n;
    // Current bank status:
    bank_status_t             status;
  } bank_state_t;

  // ======================================================================== //
  //                                                                          //
  // Combinatorial Logic                                                      //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //

  typedef enum   logic [2:0] {  ENQUEUE_FSM_IDLE  = 3'b000,
                                ENQUEUE_FSM_LOAD  = 3'b101
                                } enqueue_fsm_t;
  localparam int ENQUEUE_FSM_BUSY_B = 2;
  //
  `LIBV_REG_EN(enqueue_fsm_t, enqueue_fsm);
  `LIBV_REG_EN(bank_id_t, enqueue_bank_idx);
  `LIBV_REG_EN(addr_t, enqueue_idx);
  `LIBV_SPSRAM_SIGNALS(enqueue_, W, $clog2(N));
  //
  bank_state_t                  enqueue_bank;
  logic                                 enqueue_bank_en;
  //
  always_comb begin : enqueue_fsm_PROC

    //
    enqueue_en          = '0;
    enqueue_wen         = '0;
    enqueue_addr        = '0;
    enqueue_din         = in_dat;

    //
    enqueue_bank_idx_en  = '0;
    enqueue_bank_idx_w   = enqueue_bank_idx_r + 1'b1;

    //
    enqueue_bank         = '0;
    enqueue_bank_en      = '0;

    //
    enqueue_fsm_w        = enqueue_fsm_r;

    case (enqueue_fsm_r)

      ENQUEUE_FSM_IDLE: begin

        if (in_vld) begin
          enqueue_en          = 'b1;
          enqueue_wen         = 'b1;
          enqueue_addr        = '0;

          //
          enqueue_bank_en      = '1;
          enqueue_bank         = 0;
          enqueue_bank.status  = in_eop ? BANK_READY : BANK_LOADING;

          //
          if (!in_eop)
            enqueue_fsm_w  = ENQUEUE_FSM_LOAD;
          else
            enqueue_bank_idx_en  = 'b1;
        end
      end // case: ENQUEUE_FSM_IDLE

      ENQUEUE_FSM_LOAD: begin

        if (in_vld) begin
          enqueue_en    = 'b1;
          enqueue_wen   = 'b1;
          enqueue_addr  = addr_t'(enqueue_idx_r);

          if (in_eop) begin
            enqueue_bank_idx_en  = 'b1;

            //
            enqueue_bank         = '0;
            enqueue_bank.status  = BANK_READY;
            enqueue_bank.n       = {1'b0, enqueue_idx_r};
            enqueue_bank_en      = '1;

            //              
            enqueue_fsm_w        = ENQUEUE_FSM_IDLE;
          end
          
        end

      end // case: ENQUEUE_FSM_LOAD

      default:;

    endcase // unique case (enqueue_fsm_r)

    //
    in_rdy    = bank_idle [enqueue_bank_idx_r];

    //
    enqueue_fsm_en  = (enqueue_fsm_r [ENQUEUE_FSM_BUSY_B] |
                       enqueue_fsm_w [ENQUEUE_FSM_BUSY_B]);

    //
    enqueue_idx_en  = enqueue_fsm_en;

    //
    unique case (enqueue_fsm_r)
      ENQUEUE_FSM_IDLE:
	enqueue_idx_w  = 'b1;
      default:
        enqueue_idx_w  = enqueue_idx_r + 'b1;
    endcase // unique case (enqueue_fsm_r)

  end // block: enqueue_fsm_PROC

  // ------------------------------------------------------------------------ //
  //

  // Fetch:
  logic                                 fa_vld;
  logic 				fa_kill;
  logic 				fa_pass;
  logic 				fa_adv;

  // Decode:
  `LIBV_REG_RST(logic, da_vld, 'b0);
  logic 				da_adv;
  
  // ------------------------------------------------------------------------ //
  //
  `LIBV_REG_EN(qs_insts_pkg::pc_t, fa_pc);
  
  always_comb begin : fa_PROC
/*    
    //
    fa_pc_en    = (fa_adv | fa_kill);

    //
    case (1'b1)
      da_taken_ret:     fa_pc_w  = pc_t'(da_src1);
      da_taken_call,
      da_taken_branch:  fa_pc_w  = da_ucode.target;
      default:          fa_pc_w  = fa_pc_r + 'b1;
    endcase // case (1'b1)

    //
    fa_kill          = da_taken_branch;
    fa_valid         = (~fa_kill);
    fa_pass          = fa_valid & (~fa_kill);
    fa_adv           = fa_pass & (~da_valid_r | ~da_stall);
*/
  end // block: fetch_PROC

  // ------------------------------------------------------------------------ //
  //
  `LIBV_REG_EN(qs_insts_pkg::inst_t, da_inst);
  
  qs_ucode_rom u_qs_ucode_rom (
    //
      .ra                (fa_pc_r                 )
    //
    , .rout              (da_inst_w               )
  );

  // ------------------------------------------------------------------------ //
  //
  `LIBV_REG_EN(qs_insts_pkg::pc_t, da_pc);
  qs_insts_pkg::inst_t                  da_ucode;
  
  always_comb begin : da_PROC
/*
    //
    da_ucode  = decode(da_inst_r);


    //
    unique case (da_ucode.cc)
      EQ:      da_cc_hit  = ar_flag_z_r;
      GT:      da_cc_hit  = (~ar_flag_z_r) & (~ar_flag_n_r);
      LE:      da_cc_hit  = ar_flag_z_r | ar_flag_n_r;
      default: da_cc_hit  = 1'b1;
    endcase // unique case (da_ucode.cc)

    //
    case (da_ld_stall_r)
      1'b1:    da_ld_stall_w  = (~sort_momento_out_r);
      default: da_ld_stall_w  = da_valid_r & da_ucode.is_load;
    endcase // case (da_ld_stall_r)

    //
    priority case (1'b1)
      da_ucode.is_wait: da_stall  = (~bank_ready [sort_bank_idx_r]);
      default:          da_stall  = da_ld_stall_w;
    endcase // priority case (1'b1)
    
    //
    da_taken_branch  = da_valid_r & da_ucode.is_jump && da_cc_hit;
    da_taken_ret     = da_valid_r & da_ucode.is_ret;
    da_taken_call    = da_valid_r & da_ucode.is_call;

    //
    da_valid_w       = fa_pass & (~da_stall) | (da_valid_r & da_stall);
    da_adv           = da_valid_r & (~da_stall);
    da_en            = (~da_stall);

    //
    da_rf__ra        = {da_ucode.src1, da_ucode.src0};

    //
    case (1'b1)
      da_ucode.is_store: da_mem_op_issue  = da_valid_r;
      da_ucode.is_load:  da_mem_op_issue  = da_valid_r & (~da_ld_stall_r);
      default:           da_mem_op_issue  = '0;
    endcase // case (1'b1)
    
    //
    da_src0_is_wrbk  = da_rf__wen_r & (da_rf__ra [0] == da_rf__wa_r);
    da_src1_is_wrbk  = da_rf__wen_r & (da_rf__ra [1] == da_rf__wa_r);

    //
    da_rf__ren [0]   = da_ucode.src0_en & (~da_src0_is_wrbk);
    da_rf__ren [1]   = da_ucode.src1_en & (~da_src1_is_wrbk);

    //
    stack__cmd_vld       = da_adv & (da_ucode.is_push | da_ucode.is_pop);
    stack__cmd_push      = da_ucode.is_push;
    stack__cmd_push_dat  = da_src1;
    stack__cmd_clr       = '0;
*/
  end // block: da_PROC

  // ------------------------------------------------------------------------ //
  //
  always_comb begin : xa_PROC





  end // block: xa_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb begin : ar_PROC










  end // block: ar_PROC
    


  /*
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : exe_PROC

      //
      da_src0   = da_src0_is_wrbk ? da_rf__wdata_r : da_rf__rdata [0];
      da_src1   = da_src1_is_wrbk ? da_rf__wdata_r : da_rf__rdata [1];
      
      //
      adder__a  = da_src0 & {W{~da_ucode.src0_is_zero}};

      //
      casez ({da_ucode.has_special, da_ucode.has_imm})
        // Presently, only the CONTEXT.N special register has been
        // implemented.
        2'b1?:   adder__b_pre  = w_t'(bank_state_r [sort_bank_idx_r].n);
        2'b01:   adder__b_pre  = w_t'(da_ucode.imm);
        default: adder__b_pre  = da_src1;
      endcase // casez ({ucode.has_special, ucode.has_imm})

      //
      adder__b    = adder__b_pre ^ {W{da_ucode.inv_src1}};
      adder__cin  = da_ucode.cin;

    end // block: exe_PROC
  */

  /*
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : da_rf_PROC

      //
      da_rf__wen_w  = da_adv & da_ucode.dst_en;
      da_rf__wa_w   = da_ucode.dst;
      priority casez ({da_ucode.is_pop, da_ucode.dst_is_blink, da_ucode.is_load})
        3'b1??:  da_rf__wdata_w  = stack__cmd_pop_dat_r;
        3'b010:  da_rf__wdata_w  = w_t'(fa_pc_r);
        3'b001:  da_rf__wdata_w  = sort__dout;
        default: da_rf__wdata_w  = adder__y;
      endcase // priority casez ({ucode.is_pop, ucode.dst_is_blink})

    end // block: da_rf_PROC
  */
  /*
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : ar_flags_PROC

      //
      ar_flag_en       = da_adv & da_ucode.flag_en;
      ar_flag_c_w      = adder__cout;
      ar_flag_n_w      = adder__y [W - 1];
      ar_flag_z_w      = (adder__y == '0);

    end // block: da_flags_PROC
  */

  // ------------------------------------------------------------------------ //
  //
  //
  `LIBV_REG_EN(bank_id_t, sort_bank_idx);
  `LIBV_SPSRAM_SIGNALS(sort_, W, $clog2(N));
  bank_state_t                  sort_bank;
  logic                                 sort_bank_en;

  always_comb begin : sort_PROC
    
    //
//    sort_en          = da_mem_op_issue;
//    sort_wen         = da_ucode.is_store;
//    sort_addr        = addr_t'(da_ucode.is_store ? da_src0 : da_src1);
//    sort_din         = da_src1;

    //
//    sort_dout        = spsram_bank__dout [sort_bank_idx_r];

    // The 'momento' in this version is essentially just the rdata valid
    // as it is unnecessary to explicitly retain any state about the
    // operation.
    //
//    sort_momento_in   = sort_en & (~sort_wen);

    //
//    sort_bank_en      = da_adv & da_ucode.is_emit;
    sort_bank         = bank_state_r [sort_bank_idx_r];
    sort_bank.status  = BANK_SORTED;
    sort_bank.error   = '0;

    //
    sort_bank_idx_en  = sort_bank_en;
    sort_bank_idx_w   = sort_bank_idx_r + 'b1;

  end // block: sort_PROC

  // ------------------------------------------------------------------------ //
  //
  //
  typedef enum   logic [2:0] {  DEQUEUE_FSM_IDLE  = 3'b000,
                                DEQUEUE_FSM_EMIT  = 3'b101
                                } dequeue_fsm_t;
  localparam int DEQUEUE_FSM_BUSY_B = 2;
  //
  `LIBV_REG_EN(dequeue_fsm_t, dequeue_fsm);
  `LIBV_REG_EN(bank_id_t, dequeue_bank_idx);
  `LIBV_REG_EN(addr_t, dequeue_idx);
  `LIBV_SPSRAM_SIGNALS(dequeue_, W, $clog2(N));
  //
  bank_state_t                  dequeue_bank;
  logic                                 dequeue_bank_en;

  typedef struct packed {
    // Beat is Start-Of-Packet.
    logic                sop;
    // Beat is End-Of-Packet.
    logic                eop;
    // An Error has occurred.
    logic                err;
    // Index of nominated bank.
    bank_id_t     idx;
  } dequeue_t;

  `LIBV_REG_RST(logic, dequeue_out_vld, 'b0);
  `LIBV_REG_EN(dequeue_t, dequeue_out);
 
  always_comb begin : dequeue_fsm_PROC

    //
    dequeue_en             = 'b0;
    dequeue_wen            = 'b0;
    dequeue_addr           = 'b0;
    dequeue_din            = 'b0;

    //
    dequeue_bank_idx_en     = '0;
    dequeue_bank_idx_w      = dequeue_bank_idx_r + 'b1;

    //
    dequeue_bank            = 'b0;
    dequeue_bank_en         = 'b0;

    //
    dequeue_out_vld_w  = '0;
    dequeue_out_w.sop  = '0;
    dequeue_out_w.eop  = '0;
    dequeue_out_w.err  = '0;
    dequeue_out_w.idx  = dequeue_bank_idx_r;
    
    //
    dequeue_fsm_w           = dequeue_fsm_r;

    case (dequeue_fsm_r)

      DEQUEUE_FSM_IDLE: begin

        if (bank_sorted [dequeue_bank_idx_r]) begin
          bank_state_t st = bank_state_r [dequeue_bank_idx_r];
          
          dequeue_en 		  = 'b1;
          dequeue_addr 		  = 'b0;

          //
          dequeue_out_vld_w  = '1;
          dequeue_out_w.sop  = '1;
          dequeue_out_w.err  = st.error;

          dequeue_bank_en 	  = 'b1;
          dequeue_bank 		  = st;
          
          if (st.n == '0) begin
            dequeue_out_w.eop  = '1;

            dequeue_bank.status     = BANK_IDLE;
          end else begin
            dequeue_out_w.eop  = '0;
            
            dequeue_bank.status    = BANK_UNLOADING;
            dequeue_fsm_w          = DEQUEUE_FSM_EMIT;
          end
        end
      end

      DEQUEUE_FSM_EMIT: begin
        bank_state_t st = bank_state_r [dequeue_bank_idx_r];
        
        dequeue_en 		= 'b1;
        dequeue_addr 		= dequeue_idx_r;

        //
        dequeue_out_vld_w 	= 1'b1;
        dequeue_out_w.sop 	= 1'b0;
        dequeue_out_w.eop 	= 1'b0;
        dequeue_out_w.err 	= st.error;

        if (dequeue_idx_r == addr_t'(st.n)) begin
          dequeue_bank_idx_en = 1'b1;

          //
          dequeue_out_w.eop     = 1'b1;

          //
          dequeue_bank_en     = 1'b1;
          dequeue_bank 	      = st;
          dequeue_bank.status = BANK_IDLE;

          dequeue_fsm_w       = DEQUEUE_FSM_IDLE;
        end
        
      end // case: DEQUEUE_FSM_EMIT

      default: ;

    endcase // unique case (dequeue_fsm_r)

    //
    dequeue_fsm_en  = (dequeue_fsm_w [DEQUEUE_FSM_BUSY_B] |
                       dequeue_fsm_r [DEQUEUE_FSM_BUSY_B]);

    //
    dequeue_idx_en  = dequeue_fsm_en;

    //
    unique case (dequeue_fsm_r)
      DEQUEUE_FSM_IDLE:
	dequeue_idx_w  = 'b1;
      default:
        dequeue_idx_w  = dequeue_idx_r + 'b1;
    endcase // unique case (dequeue_fsm_r)

    // Out state latch enable
    dequeue_out_en = dequeue_out_vld_w;

  end // block: dequeue_fsm_PROC


  // ------------------------------------------------------------------------ //
  //
  bank_state_t [BANKS_N - 1:0]             bank_state_r;
  bank_state_t [BANKS_N - 1:0]             bank_state_w;
  logic [BANKS_N - 1:0]                    bank_state_en;
  //
  logic [BANKS_N - 1:0]                    bank_enqueue_upt;
  logic [BANKS_N - 1:0]                    bank_sort_upt;
  logic [BANKS_N - 1:0]                    bank_dequeue_upt;
  //
  logic [BANKS_N - 1:0]                    bank_idle;
  logic [BANKS_N - 1:0]                    bank_ready;
  logic [BANKS_N - 1:0]                    bank_sorted;

  always_comb begin : bank_state_PROC

    for (int i = 0; i < BANKS_N; i++) begin
      // Synonym for readability
      bank_id_t bank_id = bank_id_t'(i);


      // 
      bank_enqueue_upt [i] 	= enqueue_bank_en & (bank_id == enqueue_bank_idx_r);

      //
      bank_sort_upt [i] 	= sort_bank_en & (bank_id == sort_bank_idx_r);

      //
      bank_dequeue_upt [i] 	= dequeue_bank_en & (bank_id == dequeue_bank_idx_r);

      // Defaults:
      casez ({bank_enqueue_upt [i],
	      bank_sort_upt [i],
	      bank_dequeue_upt [i]
	      })
	3'b1??: begin
	  bank_state_en [i] = 'b1;
          bank_state_w [i]  = enqueue_bank;
	end
	3'b01?: begin
	  bank_state_en [i] = 'b1;
          bank_state_w [i]  = sort_bank;
	end
	3'b001: begin
	  bank_state_en [i] = 'b1;
	  bank_state_w [i]  = bank_state_r [i];
	end
	default: begin
	  bank_state_en [i] = 'b0;
	  bank_state_w [i]  = bank_state_r [i];
	end
      endcase // casez ({bank_enqueue_upt [i],...


      // TODO: deprecate; redundant.
      
      // Defaults:
      bank_idle [i]   = 'b0;
      bank_ready [i]  = 'b0;
      bank_sorted [i] = 'b0;

      case (bank_state_r [i].status)
	BANK_IDLE:   bank_idle [i]   = 'b1;
	BANK_READY:  bank_ready [i]  = 'b1;
	BANK_SORTED: bank_sorted [i] = 'b1;
	default: ;
      endcase
    end // for (int i = 0; i < BANKS_N; i++)

  end // block: bank_state_PROC
  
  // ------------------------------------------------------------------------ //
  //
  logic [BANKS_N - 1:0]                 bank_en;
  logic [BANKS_N - 1:0]                 bank_wen;
  w_t [BANKS_N - 1:0]                   bank_din;
  w_t [BANKS_N - 1:0]                   bank_dout;
  addr_t [BANKS_N - 1:0]                bank_addr;
  logic [BANKS_N - 1:0] 		bank_enqueue_sel;
  logic [BANKS_N - 1:0] 		bank_sort_sel;
  logic [BANKS_N - 1:0] 		bank_dequeue_sel;

  always_comb begin : spsram_PROC

    for (int i = 0; i < BANKS_N; i++) begin
      // Synonym for readability
      bank_id_t bank_id = bank_id_t'(i);

      // 
      bank_enqueue_sel [i] 	= enqueue_en & (bank_id == enqueue_bank_idx_r);

      //
      bank_sort_sel [i] 	= sort_en & (bank_id == sort_bank_idx_r);

      //
      bank_dequeue_sel [i] 	= dequeue_en & (bank_id == dequeue_bank_idx_r);


      casez ({// Enqueue controller maintains ownership,
	      bank_enqueue_sel [i],
	      // Or, sort controller maintains ownership,
	      bank_sort_sel [i],
	      // Or, dequeue controller maintains ownership (of current bank).
	      bank_dequeue_sel [i]
	      })
	3'b1??: begin
          bank_en [i] 	= 1'b1;
          bank_wen [i] 	= enqueue_wen;
          bank_addr [i] = enqueue_addr;
          bank_din [i] 	= enqueue_din;
	end
	3'b01?: begin
	  bank_en [i] 	= 1'b1;
	  bank_wen [i] 	= sort_wen;
	  bank_addr [i] = sort_addr;
	  bank_din [i] 	= sort_din;
	end
	3'b001: begin
          bank_en [i] 	= 1'b1;
          bank_wen [i] 	= dequeue_wen;
          bank_addr [i] = dequeue_addr;
          bank_din [i] 	= dequeue_din;
	end
	default: begin
          bank_en [i] 	= '0;
          bank_wen [i] 	= '0;
          bank_addr [i] = '0;
          bank_din [i] 	= '0;
	end
      endcase // casez ({...

    end // for (int i = 0; i < BANKS_N; i++)

  end // block: spsram_PROC

  // ------------------------------------------------------------------------ //
  //
  typedef struct packed {
    logic        sop;
    logic        eop;
    logic        err;
    w_t          dat;
  } out_t;
  
  `LIBV_REG_RST_W(logic, out_vld, 'b0);
  `LIBV_REG_EN(out_t, out);

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
  
  // ======================================================================== //
  //                                                                          //
  // Flops                                                                    //
  //                                                                          //
  // ======================================================================== //
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk) begin : bank_state_REG
    for (int i = 0; i < BANKS_N; i++) begin
      if (rst)
	bank_state_r [i] <= '{ status:BANK_IDLE, default:'0 };
      else if (bank_state_en [i])
	bank_state_r [i] <= bank_state_w [i];
    end
  end // block: bank_state_REG
  
  // ======================================================================== //
  //                                                                          //
  // Instances                                                                //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  rf #(.W(W), .N(8), .RD_N(2)) u_rf (
    //
      .clk               (clk                     )
    , .rst               (rst                     )
    //
    , .ra                ()
    , .ren               ()
    , .rdata             ()
    //
    , .wa                ()
    , .wen               ()
    , .wdata             ()
  );

  // ------------------------------------------------------------------------ //
  //
  qs_stack #(.W(W), .N(128)) u_qs_stack (
    //
      .clk               (clk                     )
    , .rst               (rst                     )
    //
    , .cmd_vld           ()
    , .cmd_push          ()
    , .cmd_push_dat      ()
    , .cmd_clr           ()
    //
    , .head_r            ()
    //
    , .cmd_err_w         ()
    //
    , .empty_w           ()
    , .full_w            ()
  );

  // ------------------------------------------------------------------------ //
  //
  generate for (genvar g = 0; g < BANKS_N; g++) begin
  
    spsram #(.W(W), .N(N)) u_bank (
      //
        .clk           (clk                       )
      //
      , .en            (bank_en [g]               )
      , .wen           (bank_wen [g]              )
      , .addr          (bank_addr [g]             )
      , .din           (bank_din [g]              )
      //
      , .dout          (bank_dout [g]             )
    );

  end endgenerate

endmodule // qs
