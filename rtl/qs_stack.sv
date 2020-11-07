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

`include "libv_pkg.vh"

module qs_stack #(parameter int N = 16, parameter int W = 32) (

   //======================================================================== //
   //                                                                         //
   // Command                                                                 //
   //                                                                         //
   //======================================================================== //

     input                                   cmd_vld
   , input                                   cmd_push
   , input        [W - 1:0]                  cmd_push_dat
   , input                                   cmd_clr
   //
   , output logic [W - 1:0]                  head_r
   //
   , output logic                            cmd_err_w

   //======================================================================== //
   //                                                                         //
   // Status                                                                  //
   //                                                                         //
   //======================================================================== //

   , output logic                            empty_w
   , output logic                            full_w

   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

   , input                                   clk
   , input                                   rst
);

  // Word type
  typedef logic [W-1:0]                 w_t;

  // Address type
  typedef logic [$clog2(N)-1:0]         addr_t;

  function automatic logic is_last_entry(addr_t addr); begin
    is_last_entry = (addr_t'(N - 1) == addr);
  end endfunction
    

  function automatic logic is_first_entry(addr_t addr); begin
    is_first_entry = (addr == '0);
  end endfunction
    

  //
  `LIBV_SPSRAM_SIGNALS(spram__, W, $clog2(N));
  //
  logic                           empty_mem_r;
  logic                           empty_mem_w;  
  //
  w_t                             cmd_pop_dat_r;
  w_t                             cmd_pop_dat_w;
  logic                           cmd_pop_dat_en;
  logic                           cmd_pop_dat_vld_r;
  logic                           cmd_pop_dat_vld_w;

  // ======================================================================== //
  //                                                                          //
  // Combinatorial Logic                                                      //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  `LIBV_REG_RST_R(logic, empty, 'b1);
  `LIBV_REG_RST_R(logic, full, 'b0);
  `LIBV_REG_EN(addr_t, rd_ptr);
  `LIBV_REG_EN(addr_t, wr_ptr);
  `LIBV_REG_RST(logic, stack_was_read, 'b0);

  `LIBV_SPSRAM_SIGNALS(stack_mem_, W, $bits(addr_t));
  
  always_comb begin : stack_PROC

    // Defaults:

    rd_ptr_en 	     = 1'b0;
    rd_ptr_w 	     = rd_ptr_r;

    wr_ptr_en 	     = 1'b0;
    wr_ptr_w 	     = wr_ptr_r;

    cmd_err_w 	     = 1'b0;

    empty_w 	     = empty_r;
    full_w 	     = full_r;

    // Stack was read in the previous cycle.
    stack_was_read_w = 'b0;

    //
    casez ({// Command is valid,
	    cmd_vld,
	    // Command type
	    cmd_push,
	    // Stack is full
	    full_r,
	    // Stack is empty
	    empty_r
	    })


      4'b1_0_?_0: begin
	// Pop from non-empty stack

	empty_w   = is_first_entry(rd_ptr_r);

	// Decrement read pointer unless stack is becoming empty.
	rd_ptr_en = (~empty_w);
	rd_ptr_w  = rd_ptr_r - 'b1;
      end
      
      4'b1_0_?_1: begin
	// Pop from empty stack (error)
	cmd_err_w = 'b1;
      end

      4'b1_1_0_?: begin
	// Push to non-full stack.
	full_w 	  = is_last_entry(wr_ptr_r);

	// Increment write pointer unless stack is becoming full.
	wr_ptr_en = (~full_w);
	wr_ptr_w  = wr_ptr_r + 'b1;
      end

      4'b1_1_1_?: begin
	// Push to full stack (error)
	cmd_err_w = 'b1;
      end

      default: ;

    endcase // casez ({...

    //
    casez ({ // Command is valid
	     cmd_vld,
	     // Command validity
	     cmd_err_w,
	     // Command type
	     cmd_push
	     })
      3'b1_0_0: begin
	// Pop command

	// Data from stack memory becomes valid in the next cycle.
	stack_was_read_w = 'b1;
	
	stack_mem_en 	 = 'b1;
	stack_mem_wen 	 = 'b0;
	stack_mem_addr 	 = rd_ptr_r;
	stack_mem_din 	 = cmd_push_dat;
      end
      3'b1_0_1: begin
	// Push command
	stack_mem_en   = 'b1;
	stack_mem_wen  = 'b1;
	stack_mem_addr = wr_ptr_r;
	stack_mem_din  = 'b0;
      end
      default: begin
	//
	stack_mem_en   = 'b0;
	stack_mem_wen  = 'b0;
	stack_mem_addr = 'b0;
	stack_mem_din  = 'b0;
      end
    endcase // casez ({...

  end // block: stack_PROC
  
  // ------------------------------------------------------------------------ //
  //
  `LIBV_REG_EN_W(w_t, head);
  
  always_comb begin : out_PROC

    // Latch output if the stack was read in the previously cycle.
    head_en = stack_was_read_r;
    head_w  = stack_mem_dout;

  end // block: out_PROC

  // ======================================================================== //
  //                                                                          //
  // Instances                                                                //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  spsram #(.W(W), .N(N), .ASYNC_DOUT('b1)) u_stack_mem (
    //
      .clk          (clk                )
    //
    , .en           (stack_mem_en       )
    , .wen          (stack_mem_wen      )
    , .addr         (stack_mem_addr     )
    , .din          (stack_mem_din      )
    //
    , .dout         (stack_mem_dout     )
  );

endmodule // qs_stack
