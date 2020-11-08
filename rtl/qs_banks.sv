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

module qs_banks (

   //======================================================================== //
   //                                                                         //
   // Enqueue                                                                 //
   //                                                                         //
   //======================================================================== //

   //
     input                                        enq_bnk_in_vld_r
   , input qs_pkg::bank_state_t                   enq_bnk_in_r
   , input qs_pkg::bank_id_t                      enq_bnk_idx_r
   //
   , output qs_pkg::bank_state_t                  enq_bnk_out_r

   //
   , input                                        enq_wr_en_r
   , input qs_pkg::addr_t                         enq_wr_addr_r
   , input qs_pkg::w_t                            enq_wr_data_r

   //======================================================================== //
   //                                                                         //
   // Sort                                                                    //
   //                                                                         //
   //======================================================================== //

   //
   , input                                        srt_bnk_in_vld_r
   , input qs_pkg::bank_state_t                   srt_bnk_in_r
   , input qs_pkg::bank_id_t                      srt_bnk_idx_r
   //
   , output qs_pkg::bank_state_t                  srt_bnk_out_r
   //
   , input                                        srt_wr_en_r
   , input qs_pkg::addr_t                         srt_wr_addr_r
   , input qs_pkg::w_t                            srt_wr_data_r
   //
   , input                                        srt_rd_en_r
   , input qs_pkg::addr_t                         srt_rd_addr_r
   //
   , output qs_pkg::w_t                           srt_rd_data_r

   //======================================================================== //
   //                                                                         //
   // Dequeue                                                                 //
   //                                                                         //
   //======================================================================== //

   //
   , input                                        deq_bnk_in_vld_r
   , input qs_pkg::bank_state_t                   deq_bnk_in_r
   , input qs_pkg::bank_id_t                      deq_bnk_idx_r
   //
   , output qs_pkg::bank_state_t                  deq_bnk_out_r

   //
   , input                                        deq_rd_en_r
   , input qs_pkg::addr_t                         deq_rd_addr_r
   //
   , output qs_pkg::w_t                           deq_rd_data_r

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
  //
  qs_pkg::bank_state_t [BANKS_N - 1:0]  bank_state_r;
  qs_pkg::bank_state_t [BANKS_N - 1:0]  bank_state_w;
  logic [BANKS_N - 1:0]                 bank_state_en;
  //
  logic [BANKS_N - 1:0]                 bank_state_enqueue_sel;
  logic [BANKS_N - 1:0]                 bank_state_sort_sel;
  logic [BANKS_N - 1:0]                 bank_state_dequeue_sel;

  // ------------------------------------------------------------------------ //
  //
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
          bank_state_w [i]  = dequeue_bank;
        end
        default: begin
          bank_state_en [i] = 'b0;
          bank_state_w [i]  = bank_state_r [i];
        end
      endcase // casez ({bank_enqueue_upt [i],...

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
*/
endmodule // qs_banks
