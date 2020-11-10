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
`include "qs_srt_pkg.vh"
`include "libv_pkg.vh"

module qs_srt (

   //======================================================================== //
   //                                                                         //
   // Bank Selection                                                          //
   //                                                                         //
   //======================================================================== //

     output qs_pkg::bank_id_t                     bank_idx_r

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
   , input                                        bank_rdata_vld_r
   , input qs_pkg::w_t                            bank_rdata_r
   //
   , output logic                                 bank_en_r
   , output logic                                 bank_wen_r
   , output qs_pkg::addr_t                        bank_addr_r
   , output qs_pkg::w_t                           bank_wdata_r

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

  // qs_stack:
  logic 		                qs_stack_cmd_vld_r;
  logic                                 qs_stack_cmd_push_r;
  qs_pkg::w_t                           qs_stack_cmd_push_dat_r;
  logic                                 qs_stack_cmd_clr_r;
  qs_pkg::w_t                           qs_stack_head_r;
  logic                                 qs_stack_head_vld_r;
  logic                                 qs_stack_cmd_err_w;
  `LIBV_REG_RST(logic, qs_stack_empty, 'b1);
  `LIBV_REG_RST(logic, qs_stack_full, 'b0);

  // rf:
  logic [1:0]                           rf_ren;
  qs_srt_pkg::reg_t [1:0]               rf_ra;
  qs_pkg::w_t [1:0]                     rf_rdata;
  //
  logic                                 rf_wen;
  qs_srt_pkg::reg_t                     rf_wa;
  qs_pkg::w_t                           rf_wdata;

  // Bank:
  `LIBV_REG_RST_W(logic, bank_en, 'b0);
  typedef struct packed {
    // Command is write
    logic 	    wen;
    // Read/Write address
    qs_pkg::addr_t  addr;
    // Write data
    qs_pkg::w_t     wdata;
  } bank_t;
  `LIBV_REG_EN(bank_t, bank);

  // Pipeline:

  // Fetch Stage (FA)
  `LIBV_REG_EN_RST(logic, fa_vld, 1'b0);
  `LIBV_REG_EN_RST(qs_srt_pkg::pc_t, fa_pc, qs_srt_pkg::RESET_VECTOR);
  logic                                 fa_stall;
  logic                                 fa_kill;
  logic                                 fa_enable;
  logic                                 fa_adv;

  // Execute Stage (XA)
  `LIBV_REG_EN_RST(logic, xa_vld, 1'b0);
  `LIBV_REG_EN(qs_srt_pkg::pc_t, xa_pc);
  `LIBV_REG_EN(qs_srt_pkg::inst_t, xa_inst);
  logic 	                        xa_enable;
  logic 	                        xa_stall_cond;
  logic 	                        xa_stall;
  logic 				xa_kill;
  logic 	                        xa_commit;
  logic 	                        xa_pass;
  logic 				xa_cc_hit;
  logic                                 xa_rf_wen;
  qs_srt_pkg::reg_t                     xa_rf_wa;
  qs_pkg::w_t                           xa_rf_wdata;
  logic                                 xa_src0_forward;
  logic                                 xa_src1_forward;
  //
  typedef struct packed {
    logic 	         pad;
    qs_pkg::w_t          w;
  } w_plus1_t;
  //
  w_plus1_t                             xa_dp_alu_0;
  w_plus1_t                             xa_dp_alu_1_pre;
  w_plus1_t                             xa_dp_alu_1;
  w_plus1_t                             xa_dp_alu_y;
  logic                                 xa_dp_alu_cout;
  //
  `LIBV_REG_RST(logic, xa_stack_cmd_vld, 'b0);

  typedef struct packed {
    logic 	    push;
    qs_pkg::w_t     push_dat;
    logic           clr;
  } xa_stack_cmd_t;

  `LIBV_REG_EN(xa_stack_cmd_t, xa_stack_cmd);

  typedef struct packed {
    // Current program counter
    qs_srt_pkg::pc_t     pc;
    // Current instruction (for debug purposes)
    qs_srt_pkg::inst_t   inst;
    // Awaiting completion of load
    logic                pending_load;
    logic 		 pending_pop;
    //
    logic                flags_en;
    // Register-file writeback
    logic                rf_wen;
    qs_srt_pkg::reg_t    rf_wa;
    qs_pkg::w_t          rf_wdata;
  } ca_ucode_t;

  // Commit Stage (CA)
  `LIBV_REG_EN_RST(logic, ca_replay, 'b0);
  `LIBV_REG_EN(qs_srt_pkg::pc_t, ca_replay_pc);
  `LIBV_REG_EN(ca_ucode_t, ca_ucode);
  `LIBV_REG_EN_RST(logic, ca_vld, 1'b0);
  logic 	                        ca_stall_cond;
  logic 	                        ca_stall;
  logic 	                        ca_enable;
  logic 				ca_pending_load;
  logic 				ca_pending_pop;
  logic 				ca_commit;

  typedef struct packed {
    logic 		      push;
    qs_pkg::w_t               push_dat;
    logic                     clr;
  } ca_stack_t;
  `LIBV_REG_RST(logic, ca_stack_cmd_vld, 'b0);
  `LIBV_REG_EN(ca_stack_t, ca_stack_cmd);

  typedef struct packed {
    // Carry bit
    logic 	 c;
    // Negative bit
    logic 	 n;
    // Zero bit
    logic 	 z;
  } ar_flags_t;
  
  `LIBV_REG_EN(ar_flags_t, ar_flags);

  // ======================================================================== //
  //                                                                          //
  // Combinatorial Logic                                                      //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  always_comb begin : fa_pipe_cntrl_PROC

    // Killed on commit stage valid.
    fa_kill   = (ca_replay_r);

    // Fetch stalls on upstream backpressure.
    fa_stall  = fa_vld_r & xa_stall & (~fa_kill);

    fa_enable = fa_kill | (~fa_stall);
    
    // Update FA PC on restart or when FA advances.
    fa_pc_en  = fa_enable;
    fa_vld_en = fa_enable;
    fa_vld_w  = 'b1;

    // Fetch stage advances.
    fa_adv    = fa_vld_r & (~fa_stall);

    // Compute next fetch program counter
    //
    casez ({// Commit-stage pipeline replay
	    ca_replay_r,
	    // Or, fetch stage advances
	    fa_adv
	    })
      2'b1?:   fa_pc_w = ca_replay_pc_r;
      2'b01:   fa_pc_w = fa_pc_r + 'b1;
      default: fa_pc_w = fa_pc_r;
    endcase // casez ({...

  end // block: fa_pipe_cntrl_PROC

  // ------------------------------------------------------------------------ //
  //
  qs_srt_ucode_rom u_qs_srt_ucode_rom (
    //
      .ra                (fa_pc_r                 )
    //
    , .rout              (xa_inst_w               )
  );

  // ------------------------------------------------------------------------ //
  //
  qs_srt_pkg::ucode_t                 xa_ucode;
  
  qs_srt_ucode_decoder u_qs_srt_ucode_decoder (
    //
      .inst              (xa_inst_r               )
    //
    , .ucode             (xa_ucode                )
  );

  // ------------------------------------------------------------------------ //
  //
  always_comb begin : xa_rf_PROC

    // Register file lookup

    //
    rf_ren [0] 	    = xa_vld_r;
    rf_ra [0] 	    = xa_ucode.src0;

    //
    rf_ren [1] 	    = xa_vld_r;
    rf_ra [1] 	    = xa_ucode.src1;

    // Forwarding (CA -> XA)
    xa_src0_forward = rf_wen & rf_ren [0] & (rf_ra [0] == rf_wa);
    xa_src1_forward = rf_wen & rf_ren [1] & (rf_ra [1] == rf_wa);

  end // block: xa_rf_PROC

  // ------------------------------------------------------------------------ //
  //
  always_comb begin : xa_datapath_PROC

    // Consider Condition Code (CC) for current instruction.
    //
    case (xa_ucode.cc)
      qs_srt_pkg::EQ: begin
	// Architectural flags compare equal.
	xa_cc_hit =   ar_flags_r.z;
      end
      qs_srt_pkg::GT: begin
	// Architectural flags compare greater-than or equal.
	xa_cc_hit = (~ar_flags_r.z) & (~ar_flags_r.n);
      end
      qs_srt_pkg::LE: begin
	// Architectural flags compare less-than or equal.
	xa_cc_hit =   ar_flags_r.z  |   ar_flags_r.n;
      end
      default: begin
	// Otherwise, CC is not considered for the current instruction.
        xa_cc_hit = 1'b1;
      end
    endcase // case (xa_ucode)

    // Compute ALU input A.
    //
    xa_dp_alu_0 = '0;
    
    casez ({// Instruction is zero; uninitialized.
	    xa_ucode.src0_is_zero,
	    // Inject BLINK
	    xa_ucode.src0_is_blink,
	    // Forward writeback
	    xa_src0_forward
	    })
      3'b1??: begin
	// Inject '0.
	xa_dp_alu_0.w = '0;
      end
      3'b01?: begin
	// Inject link address.
	xa_dp_alu_0.w = qs_pkg::w_t'(xa_pc_r) + 'b1;
      end
      3'b001: begin
	// Forward writeback
	xa_dp_alu_0.w = rf_wdata;
      end
      default: begin
	// Otherwise, select register-file state.
	xa_dp_alu_0.w = rf_rdata [0];
      end
    endcase // casez ({...

    // Compute ALU input B.
    //
    xa_dp_alu_1 = '0;
    
    casez ({// Instruction has special field.
	    xa_ucode.has_special,
	    // Inject BLINK
	    xa_ucode.is_call,
	    // Instruction has immediate
	    xa_ucode.has_imm,
	    // Forward writeback
	    xa_src1_forward
	    })
      4'b1???: begin
	// Inject "special" register.
	case (xa_ucode.special)
	  qs_srt_pkg::REG_N: begin
	    // Inject bank word count and extend as necessary.
	    xa_dp_alu_1_pre.w = qs_pkg::w_t'(bank_in.n);
	  end
	  default: begin
	    // Otherwise, unknown register. Instruction should have
	    // been flagged as invalid during initial decode.
	    xa_dp_alu_1_pre.w = '0;
	  end
	endcase
      end // case: 4'b1???
      4'b01??: begin
	// Inject link address.
	xa_dp_alu_1_pre.w = qs_pkg::w_t'(xa_pc_r) + 'b1;
      end
      4'b001?: begin
	// Inject ucode immediate field and extend as appropriate.
	xa_dp_alu_1_pre.w = qs_pkg::w_t'(xa_ucode.imm);
      end
      4'b0001: begin
	// Forward writeback
	xa_dp_alu_1_pre.w = rf_wdata;
      end
      default: begin
	// Otherwise, inject register file data.
	xa_dp_alu_1_pre.w = rf_rdata [1];
      end
    endcase // casez ({...

    // Conditionally invert ALU input, if required.
    xa_dp_alu_1   = xa_dp_alu_1_pre ^ {qs_pkg::W + 1{xa_ucode.inv_src1}};

    // Compute output of arithmetic unit.
    { xa_dp_alu_cout, xa_dp_alu_y } = 
       xa_dp_alu_0 + xa_dp_alu_1 + (xa_ucode.cin ? 'h1 : 'h0);

    // Architectural flags
    ar_flags_en  = xa_commit & xa_ucode.flag_en;

    ar_flags_w 	 = '0;
    ar_flags_w.c = xa_dp_alu_cout;
    ar_flags_w.n = xa_dp_alu_y [$left(xa_dp_alu_y)];
    ar_flags_w.z = (xa_dp_alu_y == '0);

    // Write to register file.
    xa_rf_wen 	 = xa_commit & xa_ucode.dst_en;
    xa_rf_wa 	 = xa_ucode.dst;

    // Decide value to be written back (if applicable) to the
    // architectural register file.
    //
    casez ({ // Write top of stack.
	     xa_ucode.is_pop,
	     // Write word from current bank.
	     xa_ucode.is_load
	    })
      2'b1?: begin
	// Write current stack head.
	xa_rf_wdata = qs_stack_head_r;
      end
      2'b01: begin
	// Write data returning from currently owned bank.
	xa_rf_wdata = bank_rdata_r;
      end
      default: begin
	// Otherwise, write ALU output.
	xa_rf_wdata = xa_dp_alu_y.w;
      end
    endcase // casez ({...

    // Compute replay condition which occurs on the commit of a
    // flow-control instruction (jump, ret, call). On commit, the
    // pipeline is restarted from the new program counter and old
    // instructions in the pipeline are killed.
    //
    casez ({ // Instruction commits
	     xa_commit,
	     //
	     xa_ucode.is_jump, xa_cc_hit,
	     //
	     xa_ucode.is_ret
	    })
      4'b1_11_0: begin
	// Conditional jump and condition has been met.
	ca_replay_w    = 'b1;
	ca_replay_pc_w = xa_ucode.target;
      end
      4'b1_0?_1: begin
	// RET instruction
	ca_replay_w    = 'b1;
	ca_replay_pc_w = qs_srt_pkg::pc_t'(xa_dp_alu_1);
      end
      default: begin
	ca_replay_w    = 'b0;
	ca_replay_pc_w = '0;
      end
    endcase // casez ({...
        
  end // block: xa_datapath_PROC

  // ------------------------------------------------------------------------ //
  //
  always_comb begin : xa_stack_PROC

    // Issue stack command
    ca_stack_cmd_vld_w 	    = xa_commit & (xa_ucode.is_push | xa_ucode.is_pop);

    // Stack command state
    ca_stack_cmd_en 	    = ca_stack_cmd_vld_w;
    ca_stack_cmd_w 	    = '0;
    ca_stack_cmd_w.push     = xa_ucode.is_push;
    ca_stack_cmd_w.push_dat = xa_dp_alu_1.w;

    // Unused
    ca_stack_cmd_w.clr 	    = 'b0;

    ca_pending_pop 	    = xa_ucode.is_pop;

  end // block: xa_stack_PROC
    
  // ------------------------------------------------------------------------ //
  //
  `LIBV_REG_EN(qs_pkg::bank_id_t, sort_bank_idx);
  `LIBV_SPSRAM_SIGNALS(sort_, qs_pkg::W, $clog2(qs_pkg::N));
  //
  qs_pkg::bank_state_t                  sort_bank;
  logic                                 sort_bank_en;
  //
  always_comb begin : xa_bank_PROC

    // Sort bank index update.
    sort_bank_idx_en = 'b0;
    sort_bank_idx_w  = qs_pkg::bank_id_inc(sort_bank_idx_r);

    //
    casez ({// Instruction at XA commits
	    xa_commit,
	    // Instruction is an AWAIT
	    xa_ucode.is_await,
	    // Instruction is a DONE
	    xa_ucode.is_done
	    })
      3'b1_1?: begin
	// AWAIT instruction commits; set bank status to SORTING
	sort_bank_en 	 = 'b1;
	sort_bank 	 = bank_in;
	sort_bank.status = qs_pkg::BANK_SORTING;
      end
      3'b1_01: begin
	// DONE instruction commits; set bank status to SORTED.
	sort_bank_en 	 = 'b1;
	sort_bank 	 = bank_in;
	sort_bank.status = qs_pkg::BANK_SORTED;
      end
      default: begin
	sort_bank_en     = 'b0;
	sort_bank        = bank_in;
      end
    endcase // casez ({...

    // Issue transaction to memory; subsequent command is held-up back
    // a commit stage stall (pending write-back) until the command as
    // completed.
    //
    casez ({ // Instruction commits.
	     xa_commit,
	     // Instruction is a store.
	     xa_ucode.is_store,
	     // Instruction is a load.
	     xa_ucode.is_load
	    })
      3'b1_1?: begin
	// Store
	bank_en_w 	= 'b1;

	bank_w 		= '0;
	bank_w.wen 	= 'b1;
	bank_w.addr 	= qs_pkg::addr_t'(xa_dp_alu_0);
	bank_w.wdata 	= xa_dp_alu_1.w;

	ca_pending_load = 'b0;
      end
      3'b1_01: begin
	// Load:
	bank_en_w 	= 'b1;

	bank_w 		= '0;
	bank_w.wen 	= 'b0;
	bank_w.addr 	= qs_pkg::addr_t'(xa_dp_alu_1);
	bank_w.wdata 	= xa_dp_alu_1.w;

	// Await pending load data from banks.
	ca_pending_load = 'b1;
      end
      default: begin
	// Nop
	bank_en_w 	= 'b0;
	bank_w 		= '0;

	ca_pending_load = 'b0;
      end
    endcase // casez ({...

    // 
    bank_en 	 = bank_en_w;

  end // block: xa_bank_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb begin : xa_pipe_cntrl_PROC

    // Instruction in XA is killed.
    xa_kill    = (ca_replay_r);

    // Instruction in XA is stalled.
    //
    casez ({// Current instruction is an await
	    xa_ucode.is_await
	    })
      1'b1: begin
	// Await for the current nominated stall to become ready
	xa_stall_cond = (bank_in.status != qs_pkg::BANK_READY);
      end
      default: begin
	xa_stall_cond = 'b0;
      end
    endcase // casez ({...

    // A stall occurs in the current cycle.
    xa_stall 		    = xa_vld_r & (xa_stall_cond | ca_stall) & (~xa_kill);

    // Instruction in XA commits.
    xa_commit 		    = xa_vld_r & (~xa_stall) & (~xa_kill);

    // Enable latch of new XA microcode.
    xa_enable 		    = fa_vld_r & (~xa_stall);

    xa_pass 		    = xa_vld_r & (~xa_stall) & (~xa_kill);

    //
    xa_vld_en 		    = xa_enable | xa_kill;
    xa_vld_w 		    = fa_vld_r & (~xa_kill);

    //
    xa_pc_en 		    = xa_enable;
    xa_pc_w 		    = fa_pc_r;

    // 
    xa_inst_en 		    = xa_enable;

    // Compute CA stage next state.
    ca_ucode_en 	    = ca_enable;
    
    ca_ucode_w 		    = '0;
    //
    ca_ucode_w.pc 	    = xa_pc_r;
    ca_ucode_w.inst 	    = xa_inst_r;
    //
    ca_ucode_w.pending_load = ca_pending_load;
    ca_ucode_w.pending_pop  = ca_pending_pop;
    //
    ca_ucode_w.flags_en     = ar_flags_en;
    //
    ca_ucode_w.rf_wen 	    = xa_rf_wen;
    ca_ucode_w.rf_wa 	    = xa_rf_wa;
    ca_ucode_w.rf_wdata     = xa_rf_wdata;

  end // block: xa_pipe_cntrl_PROC

  // ------------------------------------------------------------------------ //
  //
  always_comb begin : ca_pipe_cntrl_PROC

    casez ({// A load is currently pending
	    ca_ucode_r.pending_load,
	    // Load response has not yet arrived.
	    bank_rdata_vld_r,
	    // A pop is currently pending
	    ca_ucode_r.pending_pop,
	    // Pop response has not yet arrived.
	    qs_stack_head_vld_r
	    })

      4'b10_0?:
	// Stall awaiting response from memory subsystem.
	ca_stall_cond = 'b1;
      4'b0?_10:
	// Stall awaiting response from stack.
	ca_stall_cond = 'b1;
      default:
	ca_stall_cond = 'b0;
    endcase // casez ({...

    ca_stall 	    = ca_vld_r & ca_stall_cond;

    ca_commit 	    = ca_vld_r & (~ca_stall);
    
    ca_enable 	    = xa_vld_r & (~ca_stall);

    ca_vld_en 	    = ca_enable | ca_replay_r;
    ca_vld_w 	    = xa_pass;

    ca_replay_en    = ca_enable;
    ca_replay_pc_en = ca_enable;


    casez ({ // Stage is valid
	     ca_vld_r,
	     // A load is in flight
	     ca_ucode_r.pending_load,
	     // Loaded data has arrived.
	     bank_rdata_vld_r,
	     // A pop is in flight
	     ca_ucode_r.pending_pop,
	     // Pop data has arrived.
	     qs_stack_head_vld_r
	    })
      5'b1_11_??: begin
	rf_wen 	 = 'b1;
	rf_wa 	 = ca_ucode_r.rf_wa;
	rf_wdata = bank_rdata_r;
      end
      5'b1_0?_11: begin
	rf_wen 	 = 'b1;
	rf_wa 	 = ca_ucode_r.rf_wa;
	rf_wdata = qs_stack_head_r;
      end
      5'b1_0?_0?: begin
	rf_wen 	 = ca_ucode_r.rf_wen;
	rf_wa 	 = ca_ucode_r.rf_wa;
	rf_wdata = ca_ucode_r.rf_wdata;
      end
      default: begin
	rf_wen 	 = 'b0;
	rf_wa 	 = '0;
	rf_wdata = '0;
      end
    endcase // casez ({...

  end // block: ca_pipe_cntrl_PROC
  
  // ======================================================================== //
  //                                                                          //
  // Instances                                                                //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  rf #(.W(qs_pkg::W), .N(8), .RD_N(2)) u_rf (
    //
      .ra                (rf_ra                   )
    , .ren               (rf_ren                  )
    , .rdata             (rf_rdata                )
    //
    , .wa                (rf_wa                   )
    , .wen               (rf_wen                  )
    , .wdata             (rf_wdata                )
    //
    , .clk               (clk                     )
    , .rst               (rst                     )
  );

  // ------------------------------------------------------------------------ //
  //
  qs_srt_stack #(.W(qs_pkg::W), .N(qs_pkg::STACK_N)) u_qs_srt_stack (
    //
      .cmd_vld_r         (qs_stack_cmd_vld_r      )
    , .cmd_push_r        (qs_stack_cmd_push_r     )
    , .cmd_push_dat_r    (qs_stack_cmd_push_dat_r )
    , .cmd_clr_r         (qs_stack_cmd_clr_r      )
    //
    , .head_r            (qs_stack_head_r         )
    , .head_vld_r        (qs_stack_head_vld_r     )
    //
    , .cmd_err_w         (qs_stack_cmd_err_w      )
    //
    , .empty_w           (qs_stack_empty_w        )
    , .full_w            (qs_stack_full_w         )
    //
    , .clk               (clk                     )
    , .rst               (rst                     )
  );

  // ======================================================================== //
  //                                                                          //
  // Wires/Synonyms                                                           //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  always_comb begin : wires_PROC

    // Stack command interface:
    qs_stack_cmd_vld_r 	    = ca_stack_cmd_vld_r;
    qs_stack_cmd_push_r     = ca_stack_cmd_r.push;
    qs_stack_cmd_push_dat_r = ca_stack_cmd_r.push_dat;
    qs_stack_cmd_clr_r 	    = ca_stack_cmd_r.clr;

    // Bank command.
    bank_wen_r 		    = bank_r.wen;
    bank_addr_r 	    = bank_r.addr;
    bank_wdata_r 	    = bank_r.wdata;

  end // block: wires_PROC

endmodule // qs_deq
