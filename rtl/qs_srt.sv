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

module qs_srt (

   //======================================================================== //
   //                                                                         //
   // Bank Interface                                                          //
   //                                                                         //
   //======================================================================== //

   //
     input qs_pkg::bank_state_t                   bnk_in
   //
   , output logic                                 bnk_out_vld_r
   , output qs_pkg::bank_state_t                  bnk_out_r
   , output qs_pkg::bank_id_t                     bnk_idx_r
   //
   , input qs_pkg::w_t                            srt_rd_data_r
   //
   , output logic                                 srt_rd_en_r
   , output qs_pkg::addr_t                        srt_rd_addr_r
   //
   , output logic                                 srt_wr_en_r
   , output qs_pkg::addr_t                        srt_wr_addr_r
   , output qs_pkg::w_t                           srt_wr_data_r

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

  // qs_stack:
  logic 		                qs_stack_cmd_vld;
  logic                                 qs_stack_cmd_push;
  qs_pkg::w_t                           qs_stack_cmd_push_dat;
  logic                                 qs_stack_cmd_clr;
  qs_pkg::w_t                           qs_stack_head_r;
  logic                                 qs_stack_cmd_err_w;
  `LIBV_REG_RST(logic, qs_stack_empty, 'b1);
  `LIBV_REG_RST(logic, qs_stack_full, 'b0);

  // rf:
  logic [1:0]                           rf_ren;
  qs_insts_pkg::reg_t [1:0]             rf_ra;
  qs_pkg::w_t [1:0]                     rf_rdata;
  //
  logic                                 rf_wen;
  qs_insts_pkg::reg_t                   rf_wa;
  qs_pkg::w_t                           rf_wdata;

  // Pipeline:

  // Fetch Stage (FA)
  `LIBV_REG_RST(logic, fa_vld, 1'b0);
  `LIBV_REG_EN(qs_insts_pkg::pc_t, fa_pc);
  logic                                 fa_stall;
  logic                                 fa_kill;
  logic                                 fa_adv;

  // Execute Stage (XA)
  `LIBV_REG_RST(logic, xa_vld, 1'b0);
  `LIBV_REG_EN(qs_insts_pkg::pc_t, xa_pc);
  `LIBV_REG_EN(qs_insts_pkg::inst_t, xa_inst);
  logic 	                        xa_stall;
  logic 				xa_kill;
  logic 	                        xa_commit;
  logic 				xa_adv;
  logic 				xa_cc_hit;
  `LIBV_REG_RST(logic, ca_rf_wen, 'b0);
  `LIBV_REG_EN(qs_insts_pkg::reg_t, ca_rf_wa);
  `LIBV_REG_EN(qs_pkg::w_t, ca_rf_wdata);
  logic                                 xa_src0_forward;
  logic                                 xa_src1_forward;
  //
  qs_pkg::w_t                           xa_dp_alu_0;
  qs_pkg::w_t                           xa_dp_alu_1_pre;
  qs_pkg::w_t                           xa_dp_alu_1;
  logic                                 xa_dp_alu_cin;
  qs_pkg::w_t                           xa_dp_alu_y;
  logic                                 xa_dp_alu_cout;
  //
  `LIBV_REG_RST(logic, xa_stack_cmd_vld, 'b0);

  typedef struct packed {
    logic 	    push;
    qs_pkg::w_t     push_dat;
    logic           clr;
  } xa_stack_cmd_t;

  `LIBV_REG_EN(xa_stack_cmd_t, xa_stack_cmd);

  // Commit Stage (CA)
  `LIBV_REG_RST(logic, ca_replay, 'b0);
  `LIBV_REG_EN(qs_insts_pkg::pc_t, ca_replay_pc);

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
  always_comb begin : fa_PROC

    // Fetch stages on: no stall conditions
    fa_stall = fa_vld_r & (1'b0);

    // Killed on commit stage valid.
    fa_kill  = (ca_replay_r);

    // Fetch stage advances:
    fa_adv   = fa_vld_r & (~fa_kill) & ~(fa_stall | xa_stall);

    // Becomes valid after restart, or whenever not currently killed.
    fa_vld_w = ca_replay_r | (~fa_kill);
    
    // Update FA PC on restart or when FA advances.
    fa_pc_en = ca_replay_r | fa_adv;

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

  end // block: fetch_PROC

  // ------------------------------------------------------------------------ //
  //
  qs_ucode_rom u_qs_ucode_rom (
    //
      .ra                (fa_pc_r                 )
    //
    , .rout              (xa_inst_w               )
  );

  // ------------------------------------------------------------------------ //
  //
  qs_insts_pkg::ucode_t                 xa_ucode;
  
  qs_ucode_decoder u_qs_ucode_decoder (
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
    xa_src0_forward = ca_rf_wen_r & rf_ren [0] & (rf_ra [0] == ca_rf_wa_r);
    xa_src1_forward = ca_rf_wen_r & rf_ren [1] & (rf_ra [1] == ca_rf_wa_r);

  end // block: xa_rf_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb begin : xa_datapath_PROC

    // Consider Condition Code (CC) for current instruction.
    //
    case (xa_ucode.cc)
      qs_insts_pkg::EQ: begin
	// Architectural flags compare equal.
	xa_cc_hit =   ar_flags_r.z;
      end
      qs_insts_pkg::GT: begin
	// Architectural flags compare greater-than or equal.
	xa_cc_hit = (~ar_flags_r.z) & (~ar_flags_r.n);
      end
      qs_insts_pkg::LE: begin
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
    casez ({// Instruction is zero; uninitialized.
	    xa_ucode.src0_is_zero,
	    // Forward writeback
	    xa_src0_forward
	    })
      2'b1?: begin
	// Inject '0.
	xa_dp_alu_0 = '0;
      end
      2'b01: begin
	// Forward writeback
	xa_dp_alu_0 = ca_rf_wdata_r;
      end
      default: begin
	// Otherwise, select register-file state.
	xa_dp_alu_0 = rf_rdata [0];
      end
    endcase // casez ({...

    // Compute ALU input B.
    //
    casez ({// Instruction has special field.
	    xa_ucode.has_special,
	    // Instruction has immediate
	    xa_ucode.has_imm,
	    // Forward writeback
	    xa_src1_forward
	    })
      3'b1??: begin
	// Inject "special" register.
	case (xa_ucode.special)
	  qs_insts_pkg::REG_N: begin
	    // Inject bank word count and extend as necessary.
	    xa_dp_alu_1_pre = qs_pkg::w_t'(bank_state_r [sort_bank_idx_r].n);
	  end
	  default: begin
	    // Otherwise, unknown register. Instruction should have
	    // been flagged as invalid during initial decode.
	    xa_dp_alu_1_pre = '0;
	  end
	endcase
      end
      3'b01?: begin
	// Inject ucode immediate field and extend as appropriate.
	xa_dp_alu_1_pre = qs_pkg::w_t'(xa_ucode.imm);
      end
      3'b001: begin
	// Forward writeback
	xa_dp_alu_1_pre = ca_rf_wdata_r;
      end
      default: begin
	// Otherwise, inject register file data.
	xa_dp_alu_1_pre = rf_rdata [1];
      end
    endcase // casez ({...

    // Inject arithmetic unit carry-in.
    xa_dp_alu_cin = xa_ucode.cin;

    // Conditionally invert ALU input, if required.
    xa_dp_alu_1   = xa_dp_alu_1_pre ^ {W{xa_ucode.inv_src1}};

    // Compute output of arithmetic unit.
    { xa_dp_alu_cout, xa_dp_alu_y } = 
       xa_dp_alu_0 + xa_dp_alu_1 + (xa_dp_alu_cin ? 'h1 : 'h0);

    // Architectural flags
    ar_flags_en  = xa_commit & xa_ucode.flag_en;

    ar_flags_w 	 = '0;
    ar_flags_w.c = xa_dp_alu_cout;
    ar_flags_w.n = xa_dp_alu_y [$left(xa_dp_alu_y)];
    ar_flags_w.z = (xa_dp_alu_y == '0);

    // Write to register file.
    ca_rf_wen_w  = xa_commit & xa_ucode.dst_en;
    ca_rf_wa_w 	 = xa_ucode.dst;

    // Decide value to be written back (if applicable) to the
    // architectural register file.
    //
    casez ({ // Write top of stack.
	     xa_ucode.is_pop,
	     // Write link address (PC + 1)
	     xa_ucode.dst_is_blink,
	     // Write word from current bank.
	     xa_ucode.is_load
	    })
      3'b1??: begin
	// Write current stack head.
	ca_rf_wdata_w = qs_stack_head_r;
      end
      3'b01?: begin
	// Writing to the BLINK register therefore write the link
	// address for the current instruction (the next instruction).
	ca_rf_wdata_w = qs_pkg::w_t'(xa_pc_r) + 'b1;
      end
      3'b001: begin
	// Write data returning from currently owned bank.
	ca_rf_wdata_w = bank_dout [sort_bank_idx_r];
      end
      default: begin
	// Otherwise, write ALU output.
	ca_rf_wdata_w = xa_dp_alu_y;
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
	     xa_ucode.is_ret,
	     //
	     xa_ucode.is_call
	    })
      5'b1_11_?_?: begin
	// Conditional jump and condition has been met.
	ca_replay_w    = 'b1;
	ca_replay_pc_w = xa_ucode.target;
      end
      5'b1_0?_1_?: begin
	// RET instruction
	ca_replay_w    = 'b1;
	ca_replay_pc_w = qs_insts_pkg::pc_t'(xa_dp_alu_1);
      end
      5'b1_0?_0_1: begin
	// CALL instruction
	ca_replay_w    = 'b1;
	ca_replay_pc_w = qs_insts_pkg::pc_t'(xa_dp_alu_1);
      end
      default: begin
	ca_replay_w    = 'b1;
	ca_replay_pc_w = '0;
      end
    endcase // casez ({...

    // Latch replay PC on valid replay.
    ca_replay_pc_en 	  = ca_replay_w;
        
  end // block: xa_datapath_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb begin : xa_stack_PROC

    // Issue stack command
    xa_stack_cmd_vld_w 	    = xa_commit & (xa_ucode.is_push | xa_ucode.is_pop);

    // Stack command state
    xa_stack_cmd_en 	    = xa_stack_cmd_vld_w;
    xa_stack_cmd_w 	    = '0;
    xa_stack_cmd_w.push     = xa_ucode.is_push;
    xa_stack_cmd_w.push_dat = xa_dp_alu_1;

    // Unused
    xa_stack_cmd_w.clr 	    = 'b0;

  end // block: xa_stack_PROC
    
  // ------------------------------------------------------------------------ //
  //
  `LIBV_REG_EN(qs_pkg::bank_id_t, sort_bank_idx);
  `LIBV_SPSRAM_SIGNALS(sort_, W, $clog2(N));
  //
  qs_pkg::bank_state_t                  sort_bank;
  logic                                 sort_bank_en;
  //
  always_comb begin : xa_bank_PROC

    // Sort bank index update.
    sort_bank_idx_en = 'b0;
    sort_bank_idx_w  = bank_id_inc(sort_bank_idx_r);

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
	sort_bank 	 = bank_state_r [sort_bank_idx_r];
	sort_bank.status = BANK_SORTING;
      end
      3'b1_01: begin
	// DONE instruction commits; set bank status to SORTED.
	sort_bank_en 	 = 'b1;
	sort_bank 	 = bank_state_r [sort_bank_idx_r];
	sort_bank.status = BANK_SORTED;
      end
      default: begin
	sort_bank_en     = 'b0;
	sort_bank        = bank_state_r [sort_bank_idx_r];
      end
    endcase // casez ({...

    // Sort bank scratchpad memory.
    //
    sort_en   = '0;
    sort_wen  = '0;
    sort_addr = '0;
    sort_din  = '0;

  end // block: xa_bank_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb begin : xa_pipe_cntrl_PROC

    // Instruction in XA is stalled.
    //
    casez ({// Current instruction is an await
	    xa_ucode.is_await
	    })
      1'b1: begin
	// Await for the current nominated stall to become ready
	xa_stall = (bank_state_r [sort_bank_idx_r].status != BANK_READY);
      end
      default: begin
	xa_stall = 'b0;
      end
    endcase // casez ({...

    // Instruction in XA is killed.
    xa_kill   = (ca_replay_r);

    // Instruction in XA advances.
    xa_adv    = xa_vld_r & (~xa_stall) & (~xa_kill);

    // Instruction in XA commits.
    xa_commit = xa_adv;

    //
    xa_vld_w  = fa_vld_r;

    //
    xa_pc_en  = fa_adv;
    xa_pc_w   = fa_pc_r;

  end // block: xa_pipe_cntrl_PROC
  
  // ======================================================================== //
  //                                                                          //
  // Instances                                                                //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  rf #(.W(W), .N(8), .RD_N(2)) u_rf (
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
  qs_srt_stack #(.W(W), .N(128)) u_qs_srt_stack (
    //
      .cmd_vld           (qs_stack_cmd_vld        )
    , .cmd_push          (qs_stack_cmd_push       )
    , .cmd_push_dat      (qs_stack_cmd_push_dat   )
    , .cmd_clr           (qs_stack_cmd_clr        )
    //
    , .head_r            (qs_stack_head_r         )
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

    // Register file write interface:
    rf_wen 		  = ca_rf_wen_r;
    rf_wa 		  = ca_rf_wa_r;
    rf_wdata 		  = ca_rf_wdata_r;

    // Stack command interface:
    qs_stack_cmd_vld 	  = xa_stack_cmd_vld_r;
    qs_stack_cmd_push 	  = xa_stack_cmd_r.push;
    qs_stack_cmd_push_dat = xa_stack_cmd_r.push_dat;
    qs_stack_cmd_clr 	  = xa_stack_cmd_r.clr;

  end // block: wires_PROC

 */
endmodule // qs_deq
