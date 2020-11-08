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

`include "qs_insts_pkg.vh"

module qs_srt_ucode_rom (

   //======================================================================== //
   //                                                                         //
   // Lookup Interface                                                        //
   //                                                                         //
   //======================================================================== //

   // Input address
     input qs_insts_pkg::pc_t                     ra

   // Instruction
   , output qs_insts_pkg::inst_t                  rout
);
  // Import everything in this scope for readability.
  import qs_insts_pkg::*;

  // ------------------------------------------------------------------------ //
  // Algorithm:
  //
  // function partition (lo, hi) is
  //   pivot := A[hi];
  //   i := lo;
  //   for j := lo to hi - 1 do
  //     if A[j] < pivot then
  //       swap A[i] with A[j];
  //       i := i + 1;
  //   swap A[i] with A[hi];
  //   return i;
  //
  // function quicksort (lo, hi) is
  //   if lo < hi:
  //     p := partition(lo, hi)
  //     quicksort(lo, p - 1)
  //     quicksort(p + 1, hi)
  //
  //        XXXX_YYYY_YYYY_YYYY
  //  -------------------------
  //    NOP 0000_XXXX_XXXX_XXXX
  //
  //    Jcc 0001_XXcc_AAAA_AAAA
  //
  //     00 - "" Unconditional
  //     01 - "EQ" Equal
  //     10 - "GT" Greather-Than
  //     11 - "LE" Less-Than or Equal
  //
  //   PUSH 0010_0XXX_XXXX_Xuuu
  //    POP 0010_1rrr_XXXX_XXXX
  //
  //     LD 0100_0rrr_XXXX_Xuuu
  //     ST 0100_1XXX_Xsss_Xuuu
  //
  //    MOV 0110_0rrr_XXXX_0uuu
  //   MOVI 0110_0rrr_XXXX_1iii
  //   MOVS 0110_1rrr_XXXX_XSSS
  //
  //    ADD 0111_0rrr_Wsss_0uuu
  //   ADDI 0111_0rrr_Wsss_1iii
  //    SUB 0111_1rrr_Wsss_0uuu
  //   SUBI 0111_1rrr_Wsss_1iii
  //
  //   CALL 1100_0XXX_AAAA_AAAA
  //    RET 1100_1XXX_XXXX_XXXX
  //
  //   WAIT 1111_0XXX_XXXX_XXXX
  //   EMIT 1111_1XXX_XXXX_XXXX
  //
  // PROC RESET:
  //   __reset     : J __start        ; PROC RESET:
  //
  // PROC PARTITION:
  //   __part      : PUSH R2          ;
  //               : PUSH R3          ;
  //               : PUSH R4          ;
  //               : PUSH R5          ;
  //               : PUSH R6          ;
  //               : LD R2, [R1]      ; pivot <- A[hi];
  //               : MOV R3, R0       ; i <- lo
  //               : MOV R4, R0       ; j <- lo
  //   __loop_start: SUB 0, R1, R4    ;
  //               : JEQ __end        ; if (j == hi) goto __end
  //               : LD R5, [R4]      ; R5 <- A[j]
  //               : SUB.F 0, R5, R2  ;
  //               : JGT __end_loop   ; if ((A[j] - pivot) > 0) goto __end_of_loop
  //               : LD R6, [R3]      ; swap A[i] with A[j]
  //               : ST [R3], R5      ;
  //               : ST [R4], R6      ;
  //               : ADDI R3, R3, 1   ; i <- i + 1
  //   __end_loop  : ADDI R4, R4, 1   ; j <- j + 1
  //               : J __loop_start   ;
  //   __end       : LD R0, [R3]      ;
  //               : LD R1, [R4]      ;
  //               : ST [R3], R1      ;
  //               : ST [R4], R0      ;
  //               : MOV R0, R3       ; ret <- pivot
  //               : POP R6           ;
  //               : POP R5           ;
  //               : POP R4           ;
  //               : POP R3           ;
  //               : POP R2           ;
  //               : RET              ;
  //
  // PROC QUICKSORT:
  //   __qs        : PUSH BLINK       ;
  //               : PUSH R2          ;
  //               : PUSH R3          ;
  //               : PUSH R4          ;
  //               : MOV R2, R0       ; R2 <- LO
  //               : MOV R4, R1       ; R4 <- HI
  //               : SUB.F 0, R1, R0  ;
  //               : JGT __qs_end     ; if ((hi - lo) <= 0) goto __end;
  //               : CALL PARTITION   ; R0 <- partition(lo, hi);
  //               : MOV R3, R0       ; R3 <- PIVOT
  //               : MOV R0, R2       ;
  //               : SUBI R1, R3, 1   ;
  //               : CALL QUICKSORT   ; quicksort(lo, p - 1);
  //               : ADDI R0, R3, 1   ;
  //               : MOV R1, R4       ;
  //               : CALL QUICKSORT   ; quicksort(p + 1, hi);
  //   __qs_end    : POP R4           ;
  //               : POP R3           ;
  //               : POP R2           ;
  //               : POP BLINK        ; 
  //               : RET              ; PC <- BLINK
  //
  //  PROC START:
  //   __start     : AWAIT            ; wait until queue_ready == 1
  //
  //               : MOVI R0, 0       ; R0 <- "Initial LO"
  //               : MOVS R1, N       ; R1 <- "Initial HI (N)" 
  //               : CALL __qs        ; call quicksort(A, lo, hi);
  //               : DONE             ;
  //               : J __main         ; goto __main
  //

  // Reset label:
  localparam pc_t SYM_RESET = 'd0;

  // Start (entry-point) label.
  localparam pc_t SYM_START = 'd32;

  // Partition subroutine.
  localparam pc_t SYM_PARTITION = 'd64;

  // Quicksort subroutine.
  localparam pc_t SYM_QUICKSORT = 'd96;

  // Quicksort subroutine.
  localparam pc_t SYM_ERR = 'd128;

  always_comb begin : quicksort_prog_PROC

    // Control Store
    //
    // Implemented here as a simple lookup table, but in practise more
    // more efficiently realized as a ROM. Perhaps an FPGA synthesis
    // tool can automatically infer a ROM from this table, otherwise, the
    // microcode would need to be hand assembled and loaded as a HEX-file.
    
    case (ra)
      // Reset vector
      SYM_RESET          : rout = j(SYM_START);

      // Parition subroutine entry point
      SYM_PARTITION      : rout = push(R2);
      SYM_PARTITION +   1: rout = push(R3);
      SYM_PARTITION +   2: rout = push(R4);
      SYM_PARTITION +   3: rout = push(R5);
      SYM_PARTITION +   4: rout = push(R6);
      SYM_PARTITION +   5: rout = ld(R2, R1);
      SYM_PARTITION +   6: rout = mov(R3, R0);
      SYM_PARTITION +   7: rout = mov(R4, R0);
      SYM_PARTITION +   8: rout = sub(R0, R1, R4, .dst_en('b0));
      SYM_PARTITION +   9: rout = j(SYM_PARTITION + 19, .cc(EQ));
      SYM_PARTITION +  10: rout = ld(R5, R4);
      SYM_PARTITION +  11: rout = sub(R0, R5, R2, .dst_en('b0));
      SYM_PARTITION +  12: rout = j(SYM_PARTITION + 17, .cc(GT));
      SYM_PARTITION +  13: rout = ld(R6, R3);
      SYM_PARTITION +  14: rout = st(R3, R5);
      SYM_PARTITION +  15: rout = st(R4, R6);
      SYM_PARTITION +  16: rout = addi(R3, R3, 'b1);
      SYM_PARTITION +  17: rout = addi(R4, R4, 'b1);
      SYM_PARTITION +  18: rout = j(SYM_PARTITION + 8);
      SYM_PARTITION +  19: rout = ld(R0, R3);
      SYM_PARTITION +  20: rout = ld(R1, R4);
      SYM_PARTITION +  21: rout = st(R3, R1);
      SYM_PARTITION +  22: rout = st(R4, R0);
      SYM_PARTITION +  23: rout = mov(R0, R3);
      SYM_PARTITION +  24: rout = pop(R6);
      SYM_PARTITION +  25: rout = pop(R5);
      SYM_PARTITION +  26: rout = pop(R4);
      SYM_PARTITION +  27: rout = pop(R3);
      SYM_PARTITION +  28: rout = pop(R2);
      SYM_PARTITION +  29: rout = ret();

      // Quicksort subroutine entry point
      SYM_QUICKSORT      : rout = push(BLINK);
      SYM_QUICKSORT +   1: rout = push(R2);
      SYM_QUICKSORT +   2: rout = push(R3);
      SYM_QUICKSORT +   3: rout = push(R4);
      SYM_QUICKSORT +   4: rout = mov(R2, R0);
      SYM_QUICKSORT +   5: rout = mov(R4, R1);
      SYM_QUICKSORT +   6: rout = sub(R0, R0, R1, .dst_en('b0));
      SYM_QUICKSORT +   7: rout = j(SYM_QUICKSORT + 16, GT);
      SYM_QUICKSORT +   8: rout = call(SYM_PARTITION);
      SYM_QUICKSORT +   9: rout = mov(R3, R0);
      SYM_QUICKSORT +  10: rout = mov(R0, R2);
      SYM_QUICKSORT +  11: rout = subi(R1, R3, 'd1);
      SYM_QUICKSORT +  12: rout = call(SYM_QUICKSORT);
      SYM_QUICKSORT +  13: rout = addi(R0, R3, 'd1);
      SYM_QUICKSORT +  14: rout = mov(R1, R4);
      SYM_QUICKSORT +  15: rout = call(SYM_QUICKSORT);
      SYM_QUICKSORT +  16: rout = pop(R4);
      SYM_QUICKSORT +  17: rout = pop(R3);
      SYM_QUICKSORT +  18: rout = pop(R2);
      SYM_QUICKSORT +  19: rout = pop(BLINK);
      SYM_QUICKSORT +  20: rout = ret();

      // Algorithm start label.
      SYM_START          : rout = await();
      SYM_START +       1: rout = movi(R0, '0);
      SYM_START +       2: rout = movs(R1, REG_N);
      SYM_START +       3: rout = call(SYM_QUICKSORT);
      SYM_START +       4: rout = done();
      SYM_START +       5: rout = j(SYM_START);

      // Error label (jump to self for all enternity).
      SYM_ERR            : rout = j(SYM_ERR);
            
      default:             rout = j(SYM_ERR);
      
    endcase // case (pa)

  end // block: quicksort_prog_PROC

endmodule // qs_srt_ucode_rom
