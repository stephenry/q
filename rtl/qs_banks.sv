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
     input qs_pkg::bank_id_t                      enq_bank_idx_r
   //
   , input                                        enq_bank_in_vld
   , input qs_pkg::bank_state_t                   enq_bank_in
   , output qs_pkg::bank_state_t                  enq_bank_out
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
   , input qs_pkg::bank_id_t                      srt_bank_idx_r
   //
   , input                                        srt_bank_in_vld
   , input qs_pkg::bank_state_t                   srt_bank_in
   , output qs_pkg::bank_state_t                  srt_bank_out
   //
   , input                                        srt_bank_en_r
   , input                                        srt_bank_wen_r
   , input qs_pkg::addr_t                         srt_bank_addr_r
   , input qs_pkg::w_t                            srt_bank_wdata_r
   //
   , output logic                                 srt_bank_rdata_vld_r
   , output qs_pkg::w_t                           srt_bank_rdata_r

   //======================================================================== //
   //                                                                         //
   // Dequeue                                                                 //
   //                                                                         //
   //======================================================================== //

   , input qs_pkg::bank_id_t                      deq_bank_idx_r
   //
   , input                                        deq_bank_in_vld
   , input qs_pkg::bank_state_t                   deq_bank_in
   , output qs_pkg::bank_state_t                  deq_bank_out
   //
   , input                                        deq_rd_en_r
   , input qs_pkg::addr_t                         deq_rd_addr_r
   //
   , output logic                                 deq_rd_data_vld_r
   , output qs_pkg::w_t                           deq_rd_data_r

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
  //
  qs_pkg::bank_state_t [qs_pkg::BANKS_N - 1:0]  bank_state_r;
  qs_pkg::bank_state_t [qs_pkg::BANKS_N - 1:0]  bank_state_w;
  logic [qs_pkg::BANKS_N - 1:0]                 bank_state_en;
  //
  logic [qs_pkg::BANKS_N - 1:0]                 bank_state_enq_sel;
  logic [qs_pkg::BANKS_N - 1:0]                 bank_state_srt_sel;
  logic [qs_pkg::BANKS_N - 1:0]                 bank_state_deq_sel;

  // ------------------------------------------------------------------------ //
  //
  always_comb begin : bank_state_PROC

    for (int i = 0; i < qs_pkg::BANKS_N; i++) begin
      // 
      bank_state_enq_sel [i] =
        enq_bank_in_vld & (qs_pkg::bank_id_t'(i) == enq_bank_idx_r);

      //
      bank_state_srt_sel [i]    =
        srt_bank_in_vld & (qs_pkg::bank_id_t'(i) == srt_bank_idx_r);

      //
      bank_state_deq_sel [i] =
        deq_bank_in_vld & (qs_pkg::bank_id_t'(i) == deq_bank_idx_r);

      // Defaults:
      casez ({// Enqueue controller updates bank state
	      bank_state_enq_sel [i],
	      // Sort controller updates bank state
              bank_state_srt_sel [i],
	      // Dequeue controller updates bank state
              bank_state_deq_sel [i]
              })
        3'b1??: begin
          bank_state_en [i] = 'b1;
          bank_state_w [i]  = enq_bank_in;
        end
        3'b01?: begin
          bank_state_en [i] = 'b1;
          bank_state_w [i]  = srt_bank_in;
        end
        3'b001: begin
          bank_state_en [i] = 'b1;
          bank_state_w [i]  = deq_bank_in;
        end
        default: begin
          bank_state_en [i] = 'b0;
          bank_state_w [i]  = bank_state_r [i];
        end
      endcase // casez ({bank_enq_upt [i],...

    end // for (int i = 0; i < qs_pkg::BANKS_N; i++)

    // Emit bank state to clients.
    enq_bank_out      = bank_state_r [enq_bank_idx_r];
    srt_bank_out      = bank_state_r [srt_bank_idx_r];
    deq_bank_out      = bank_state_r [deq_bank_idx_r];

  end // block: bank_state_PROC

  // ------------------------------------------------------------------------ //
  //
  logic [qs_pkg::BANKS_N - 1:0]                 bank_en;
  logic [qs_pkg::BANKS_N - 1:0]                 bank_wen;
  qs_pkg::w_t [qs_pkg::BANKS_N - 1:0]           bank_din;
  qs_pkg::w_t [qs_pkg::BANKS_N - 1:0]           bank_dout;
  qs_pkg::addr_t [qs_pkg::BANKS_N - 1:0]        bank_addr;
  //
  logic [qs_pkg::BANKS_N - 1:0]                 bank_enq_sel;
  logic [qs_pkg::BANKS_N - 1:0]                 bank_srt_sel;
  logic [qs_pkg::BANKS_N - 1:0]                 bank_deq_sel;
  `LIBV_REG_RST(logic, bank_deq_rd_vld, 'b0);
  `LIBV_REG_EN(qs_pkg::bank_id_t, bank_deq_rd_idx);
  `LIBV_REG_RST(logic, bank_srt_rd_vld, 'b0);
  `LIBV_REG_EN(qs_pkg::bank_id_t, bank_srt_rd_idx);

  always_comb begin : bank_PROC

    // Defaults:
    bank_deq_rd_vld_w  = 'b0;

    bank_deq_rd_idx_en = 'b0;
    bank_deq_rd_idx_w  = bank_deq_rd_idx_r;
        
    bank_srt_rd_vld_w  = 'b0;

    bank_srt_rd_idx_en = 'b0;
    bank_srt_rd_idx_w  = bank_srt_rd_idx_r;

    for (int i = 0; i < qs_pkg::BANKS_N; i++) begin
      // 
      bank_enq_sel [i] =
        enq_wr_en_r & (qs_pkg::bank_id_t'(i) == enq_bank_idx_r);

      //
      bank_srt_sel [i]    =
        srt_bank_en_r & (qs_pkg::bank_id_t'(i) == srt_bank_idx_r);

      //
      bank_deq_sel [i] =
        deq_rd_en_r & (qs_pkg::bank_id_t'(i) == deq_bank_idx_r);


      casez ({// Enq controller maintains ownership,
              bank_enq_sel [i],
              // Or, srt controller maintains ownership,
              bank_srt_sel [i],
              // Or, dequeue controller maintains ownership (of current bank).
              bank_deq_sel [i]
              })
        3'b1??: begin
          bank_en [i]   = 1'b1;
          bank_wen [i]  = 1'b1;
          bank_addr [i] = enq_wr_addr_r;
          bank_din [i]  = enq_wr_data_r;
        end
        3'b01?: begin
	  bank_srt_rd_vld_w  = 'b1;
	  bank_srt_rd_idx_en = 'b1;
	  bank_srt_rd_idx_w  = srt_bank_idx_r;
	  
          bank_en [i] 	     = 1'b1;
          bank_wen [i] 	     = srt_bank_wen_r;
          bank_addr [i]      = srt_bank_addr_r;
          bank_din [i] 	     = srt_bank_wdata_r;
        end
        3'b001: begin
	  bank_deq_rd_vld_w  = 'b1;
	  bank_deq_rd_idx_en = 'b1;
	  bank_deq_rd_idx_w  = deq_bank_idx_r;
	  
          bank_en [i] 	     = 1'b1;
          bank_wen [i] 	     = 1'b0;
          bank_addr [i]      = deq_rd_addr_r;
          bank_din [i] 	     = '0;
        end
        default: begin
          bank_en [i]   = '0;
          bank_wen [i]  = '0;
          bank_addr [i] = '0;
          bank_din [i]  = '0;
        end
      endcase // casez ({...

    end // for (int i = 0; i < qs_pkg::BANKS_N; i++)

  end // block: bank_PROC

  // ------------------------------------------------------------------------ //
  //
  `LIBV_REG_RST_W(logic, srt_bank_rdata_vld, 1'b0);
  `LIBV_REG_EN_W(qs_pkg::w_t, srt_bank_rdata);
  `LIBV_REG_RST_W(logic, deq_rd_data_vld, 'b0);
  `LIBV_REG_EN_W(qs_pkg::w_t, deq_rd_data);

  always_comb begin : out_PROC

    srt_bank_rdata_vld_w = bank_srt_rd_vld_r;
    srt_bank_rdata_en 	 = bank_srt_rd_vld_r;
    srt_bank_rdata_w 	 = bank_dout [bank_srt_rd_idx_r];

    deq_rd_data_vld_w    = bank_deq_rd_vld_r;
    deq_rd_data_en 	 = bank_deq_rd_vld_r;
    deq_rd_data_w 	 = bank_dout [bank_deq_rd_idx_r];

  end // block: out_PROC
  
  // ======================================================================== //
  //                                                                          //
  // Flops                                                                    //
  //                                                                          //
  // ======================================================================== //
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk) begin : bank_state_REG
    for (int i = 0; i < qs_pkg::BANKS_N; i++) begin
      if (rst)
        bank_state_r [i] <= '{ status:qs_pkg::BANK_IDLE, default:'0 };
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
  generate for (genvar g = 0; g < qs_pkg::BANKS_N; g++) begin
  
    spsram #(.W(qs_pkg::W), .N(qs_pkg::N)) u_bank (
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

  end endgenerate // for (genvar g = 0; g < qs_pkg::BANKS_N; g++)

endmodule // qs_banks
