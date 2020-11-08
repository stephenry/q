//========================================================================== //
// Copyright (c) 2018, Stephen Henry
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

`include "qs_srt_pkg.vh"

module qs_srt_ucode_decoder (

   //======================================================================== //
   //                                                                         //
   // Decode Interface                                                        //
   //                                                                         //
   //======================================================================== //

     input qs_srt_pkg::inst_t                   inst
   //
   , output qs_srt_pkg::ucode_t                 ucode
);

  // Import everything into this scope for readability
  import qs_srt_pkg::*;

  // ------------------------------------------------------------------------ //
  //
  logic                                 decode_sel;
  
  always_comb begin : decode_PROC

    // Defaults:
    ucode 	  = '0;

    decode_sel 	  = SEL_field(inst);

    // COMMON
    ucode.imm 	  = I_field(inst);
    ucode.dst 	  = R_field(inst);
    ucode.src0 	  = S_field(inst);
    ucode.src1 	  = U_field(inst);
    ucode.special = SPECIAL_field(inst);
    ucode.cc 	  = CC_field(inst);
    ucode.target  = A_field(inst);
    
    case (inst.opcode)
      NOP: begin
      end
      JCC: begin
        // Jcc
        ucode.is_jump  = 'b1;
      end
      PP: begin
        case (decode_sel)
          1'b1: begin
            // POP
            ucode.is_pop  = 'b1;
            ucode.dst_en  = 'b1;
          end
          default: begin
            // PUSH
            ucode.is_push       = 'b1;
            ucode.src1_en       = 'b1;
            ucode.src0_is_zero  = 'b1;
          end
        endcase
      end
      MEM: begin
        ucode.src1_en   = 'b1;
        case (decode_sel)
          1'b1: begin
            // ST
            ucode.is_store  = 'b1;
            ucode.src0_en   = 'b1;
          end
          default: begin
            // LD
            ucode.is_load  = 'b1;
            ucode.dst_en   = 'b1;
          end
        endcase // case (decode_sel)
      end
      MOV: begin
        ucode.dst_en        = 'b1;
        ucode.src0_is_zero  = 'b1;
        unique0 casez ({inst[11],inst[3]})
          // MOV
          2'b00:   ucode.src1_en      = 'b1;
          // MOVI
          2'b01:   ucode.has_imm      = 'b1;
          // MOVS
          2'b1?:   ucode.has_special  = 'b1;
        endcase // unique0 casez ({inst[11],inst[3]})
      end
      ARITH: begin
        ucode.dst_en   = W_field(inst);
        ucode.flag_en  = 'b1;
        ucode.src0_en  = 'b1;
        if (IMM_field(inst))
          // {ADD,SUB}I
          ucode.has_imm  = 'b1;
        else
          // {ADD,SUB}
          ucode.src1_en  = 'b1;
        if (decode_sel) begin
          // SUB
          ucode.inv_src1  = 'b1;
          ucode.cin       = 'b1;
        end
      end
      CRET: begin
        ucode.is_jump       = 'b1;
        ucode.src0_is_zero  = 'b1;
        case (decode_sel)
          1'b1: begin
            // RET
            ucode.is_ret   = 'b1;
            ucode.src1_en  = 'b1;
            ucode.src1     = BLINK;
          end
          default: begin
            // CALL
            ucode.is_call  = 'b1;
            ucode.dst_en   = 'b1;
            ucode.dst      = BLINK;
          end
        endcase // case (inst.u.cret.is_call)
      end
      CNTRL: begin
        if (decode_sel)
          ucode.is_done  = 'b1;
        else
          ucode.is_await = 'b1;
      end
      default: begin
        ucode.invalid_inst  = 'b1;
      end
    endcase // case (inst.opcode)

    //
    ucode.dst_is_blink  = (ucode.dst == BLINK);

  end // block: decode_PROC

endmodule // qs_srt_ucode_decoder
