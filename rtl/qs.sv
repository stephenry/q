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
     input                                        in_vld
   , input                                        in_sop
   , input                                        in_eop
   , input [W - 1:0]                              in_dat
   //
   , output logic                                 in_rdy

   //======================================================================== //
   //                                                                         //
   // Sorted                                                                  //
   //                                                                         //
   //======================================================================== //

   //
   , output logic                                 out_vld_r
   , output logic                                 out_sop_r
   , output logic                                 out_eop_r
   , output logic                                 out_err_r
   , output logic [W - 1:0]                       out_dat_r

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
  // Types                                                                    //
  //                                                                          //
  // ======================================================================== //

  // Word type:
  typedef logic [W - 1:0]               w_t;

  // Sort vector addresss:
  typedef logic [$clog2(N) - 1:0]       addr_t;

  // Type to represent the number of entries in a vector.
  typedef logic [$clog2(N):0]           n_t;

  // Type to represent the index of a bank.
  typedef logic [$clog2(BANKS_N) - 1:0] bank_id_t;

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
    logic                     error;
    // Current bank word count.
    n_t                       n;
    // Current bank status:
    bank_status_t             status;
  } bank_state_t;

  // ======================================================================== //
  //                                                                          //
  // Helper functions                                                         //
  //                                                                          //
  // ======================================================================== //

  function automatic bank_id_t bank_id_inc(bank_id_t id); begin
    logic [$clog2(BANKS_N):0] sum = id + 'b1;

    bank_id_inc = (sum == N[$clog2(BANKS_N):0]) ? '0 : bank_id_t'(sum);
  end endfunction

  // ======================================================================== //
  //                                                                          //
  // Wires                                                                    //
  //                                                                          //
  // ======================================================================== //

  // qs_stack:
  logic 		                qs_stack_cmd_vld;
  logic                                 qs_stack_cmd_push;
  w_t                                   qs_stack_cmd_push_dat;
  logic                                 qs_stack_cmd_clr;
  w_t                                   qs_stack_head_r;
  logic                                 qs_stack_cmd_err_w;
  `LIBV_REG_RST(logic, qs_stack_empty, 'b1);
  `LIBV_REG_RST(logic, qs_stack_full, 'b0);

  // rf:
  logic [1:0]                           rf_ren;
  qs_insts_pkg::reg_t [1:0]             rf_ra;
  w_t [1:0]                             rf_rdata;
  //
  logic                                 rf_wen;
  qs_insts_pkg::reg_t                   rf_wa;
  w_t                                   rf_wdata;

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
  // Enqueue state machine state encodings.
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
  logic                         enqueue_bank_en;
  //
  always_comb begin : enqueue_fsm_PROC

    //
    enqueue_en 		= '0;
    enqueue_wen 	= '0;
    enqueue_addr 	= '0;
    enqueue_din 	= in_dat;

    //
    enqueue_bank_idx_en = '0;
    enqueue_bank_idx_w 	= enqueue_bank_idx_r + 1'b1;

    //
    enqueue_bank 	= '0;
    enqueue_bank_en 	= '0;

    //
    enqueue_fsm_w 	= enqueue_fsm_r;

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
  w_t                                   alu_a;
  w_t                                   alu_b;
  logic                                 alu_cin;
  w_t                                   alu_y;
  logic                                 alu_cout;
  
  always_comb begin : xa_PROC

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
    endcase

    // Instruction in XA is killed.
    xa_kill   = (ca_replay_r);

    // Instruction in XA advances.
    xa_adv    = xa_vld_r & (~xa_stall) & (~xa_kill);

    // Instruction in XA commits.
    xa_commit = xa_adv;

    //
    xa_vld_w = fa_vld_r;


    //
    xa_pc_en = 'b0;
    xa_pc_w  = fa_pc_r;

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


    casez ({1'b0})
      default: alu_a = '0;
    endcase // casez ({})


    casez ({1'b0})
      default: alu_b = '0;
    endcase // casez ({})

    // Inject arithmetic unit carry-in.
    alu_cin 		= xa_ucode.cin;

    // Compute output of arithmetic unit.
    { alu_cout, alu_y } = alu_a + alu_b + (alu_cin ? 'h1 : 'h0);

    // Architectural flags
    ar_flags_en 	= xa_commit;

    ar_flags_w 		= '0;
    ar_flags_w.c 	= alu_cout;
    ar_flags_w.n 	= alu_y [$left(alu_y)];
    ar_flags_w.z 	= (alu_y == '0);

    // Write to register file.
    rf_wen 		= xa_commit & xa_ucode.dst_en;
    rf_wa 		= xa_ucode.dst;


    // Decide value to be written back (if applicable) to the
    // architectural register file.
    //
    casez ({ // Write top of stack.
	     xa_ucode.is_pop,
	     // Write current architectural program counter.
	     xa_ucode.dst_is_blink,
	     // Write word from current bank.
	     xa_ucode.is_load
	    })
      3'b1??: begin
	//
	rf_wdata = qs_stack_head_r;
      end
      3'b01?: begin
	//
	rf_wdata = '0;
      end
      3'b001: begin
	//
	rf_wdata = '0;
      end
      default: begin 
	rf_wdata = alu_y;
      end
    endcase

    ca_replay_w    = 1'b0;
    ca_replay_pc_w = '0;

    // Enables:
    ca_replay_pc_en 	= ca_replay_w;
    
  end // block: xa_PROC
  
  // ------------------------------------------------------------------------ //
  //
  `LIBV_REG_EN(bank_id_t, sort_bank_idx);
  `LIBV_SPSRAM_SIGNALS(sort_, W, $clog2(N));
  //
  bank_state_t                          sort_bank;
  logic                                 sort_bank_en;
  //
  always_comb begin : sort_bank_PROC

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

  end // block: sort_bank_PROC

  // ------------------------------------------------------------------------ //
  //
  // Dequeue state machine state encodings.
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
  logic                         dequeue_bank_en;

  typedef struct packed {
    // Beat is Start-Of-Packet.
    logic                sop;
    // Beat is End-Of-Packet.
    logic                eop;
    // An Error has occurred.
    logic                err;
    // Index of nominated bank.
    bank_id_t            idx;
  } dequeue_t;

  `LIBV_REG_RST(logic, dequeue_out_vld, 'b0);
  `LIBV_REG_EN(dequeue_t, dequeue_out);
 
  always_comb begin : dequeue_fsm_PROC

    //
    dequeue_en 		= 'b0;
    dequeue_wen 	= 'b0;
    dequeue_addr 	= 'b0;
    dequeue_din 	= 'b0;

    //
    dequeue_bank_idx_en = '0;
    dequeue_bank_idx_w 	= dequeue_bank_idx_r + 'b1;

    //
    dequeue_bank 	= 'b0;
    dequeue_bank_en 	= 'b0;

    //
    dequeue_out_vld_w 	= '0;
    dequeue_out_w.sop 	= '0;
    dequeue_out_w.eop 	= '0;
    dequeue_out_w.err 	= '0;
    dequeue_out_w.idx 	= dequeue_bank_idx_r;
    
    //
    dequeue_fsm_w 	= dequeue_fsm_r;

    case (dequeue_fsm_r)

      DEQUEUE_FSM_IDLE: begin

        if (bank_sorted [dequeue_bank_idx_r]) begin
          bank_state_t st   = bank_state_r [dequeue_bank_idx_r];
          
          dequeue_en 	    = 'b1;
          dequeue_addr 	    = 'b0;

          //
          dequeue_out_vld_w = '1;
          dequeue_out_w.sop = '1;
          dequeue_out_w.err = st.error;

          dequeue_bank_en   = 'b1;
          dequeue_bank 	    = st;
          
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
        
        dequeue_en              = 'b1;
        dequeue_addr            = dequeue_idx_r;

        //
        dequeue_out_vld_w       = 1'b1;
        dequeue_out_w.sop       = 1'b0;
        dequeue_out_w.eop       = 1'b0;
        dequeue_out_w.err       = st.error;

        if (dequeue_idx_r == addr_t'(st.n)) begin
          dequeue_bank_idx_en = 1'b1;

          //
          dequeue_out_w.eop     = 1'b1;

          //
          dequeue_bank_en     = 1'b1;
          dequeue_bank        = st;
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
  logic [BANKS_N - 1:0]                    bank_state_enqueue_sel;
  logic [BANKS_N - 1:0]                    bank_state_sort_sel;
  logic [BANKS_N - 1:0]                    bank_state_dequeue_sel;
  //
  logic [BANKS_N - 1:0]                    bank_idle;
  logic [BANKS_N - 1:0]                    bank_ready;
  logic [BANKS_N - 1:0]                    bank_sorted;

  always_comb begin : bank_state_PROC

    for (int i = 0; i < BANKS_N; i++) begin
      // Synonym for readability
      bank_id_t bank_id = bank_id_t'(i);

      // 
      bank_state_enqueue_sel [i] =
        enqueue_bank_en & (bank_id == enqueue_bank_idx_r);

      //
      bank_state_sort_sel [i]    =
        sort_bank_en & (bank_id == sort_bank_idx_r);

      //
      bank_state_dequeue_sel [i] =
        dequeue_bank_en & (bank_id == dequeue_bank_idx_r);

      // Defaults:
      casez ({// Enqueue controller updates bank state
	      bank_state_enqueue_sel [i],
	      // Sort controller updates bank state
              bank_state_sort_sel [i],
	      // Dequeue controller updates bank state
              bank_state_dequeue_sel [i]
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
          bank_state_w [i]  = bank_state_r [i]; // WTF?
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
  logic [BANKS_N - 1:0]                 bank_enqueue_sel;
  logic [BANKS_N - 1:0]                 bank_sort_sel;
  logic [BANKS_N - 1:0]                 bank_dequeue_sel;

  always_comb begin : bank_PROC

    for (int i = 0; i < BANKS_N; i++) begin
      // Synonym for readability
      bank_id_t bank_id = bank_id_t'(i);

      // 
      bank_enqueue_sel [i] =
        enqueue_en & (bank_id == enqueue_bank_idx_r);

      //
      bank_sort_sel [i]    =
        sort_en & (bank_id == sort_bank_idx_r);

      //
      bank_dequeue_sel [i] =
        dequeue_en & (bank_id == dequeue_bank_idx_r);


      casez ({// Enqueue controller maintains ownership,
              bank_enqueue_sel [i],
              // Or, sort controller maintains ownership,
              bank_sort_sel [i],
              // Or, dequeue controller maintains ownership (of current bank).
              bank_dequeue_sel [i]
              })
        3'b1??: begin
          bank_en [i]   = 1'b1;
          bank_wen [i]  = enqueue_wen;
          bank_addr [i] = enqueue_addr;
          bank_din [i]  = enqueue_din;
        end
        3'b01?: begin
          bank_en [i]   = 1'b1;
          bank_wen [i]  = sort_wen;
          bank_addr [i] = sort_addr;
          bank_din [i]  = sort_din;
        end
        3'b001: begin
          bank_en [i]   = 1'b1;
          bank_wen [i]  = dequeue_wen;
          bank_addr [i] = dequeue_addr;
          bank_din [i]  = dequeue_din;
        end
        default: begin
          bank_en [i]   = '0;
          bank_wen [i]  = '0;
          bank_addr [i] = '0;
          bank_din [i]  = '0;
        end
      endcase // casez ({...

    end // for (int i = 0; i < BANKS_N; i++)

  end // block: bank_PROC

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
    , .ra                (rf_ra                   )
    , .ren               (rf_ren                  )
    , .rdata             (rf_rdata                )
    //
    , .wa                (rf_wa                   )
    , .wen               (rf_wen                  )
    , .wdata             (rf_wdata                )
  );

  // ------------------------------------------------------------------------ //
  //
  qs_stack #(.W(W), .N(128)) u_qs_stack (
    //
      .clk               (clk                     )
    , .rst               (rst                     )
    //
    , .cmd_vld           (qs_stack_cmd_vld        )
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

  end endgenerate // for (genvar g = 0; g < BANKS_N; g++)

endmodule // qs
