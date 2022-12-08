// ========================================================================= //
// Copyright (c) 2022, Stephen Henry
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
// SUBSTITUTE GOODS OR SERVICES/ LOSS OF USE, DATA, OR PROFITS/ OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// ========================================================================= //

    .org 0
reset:
    j __start                   // reset vector, jump to main subroutine.

    .org 16
partition:
    push r2                     //
    push r3                     //
    push r4                     //
    push r5                     //
    push r6                     //
    ld x0, [r1]                 // pivot <- A[hi]//
    mov r3, r0                  // i <- lo
    mov r4, r0                  // j <- lo   
__loop_start:
    cmp r1, r4                  //
    jeq __end                   // if (j == hi) goto __end
    ld x1, [r4]                 // r5 <- A[j]
    cmp x1, x0                  //
    jgt __end_loop              // if ((A[j] - pivot) > 0) goto __end_of_loop
    ld x2, [r3]                 // swap A[i] with A[j]
    st [r3], x1                 //
    st [r4], x2                 //
    addi r3, r3, 1              // i <- i + 1
__end_loop:
    addi r4, r4, 1              // j <- j + 1
    j __loop_start              //
__end:
    ld x0, [r3]                 //
    ld x1, [r4]                 //
    st [r3], x1                 //
    st [r4], x0                 //
    mov r0, r3                  // ret <- pivot
    pop r6                      //
    pop r5                      //
    pop r4                      //
    pop r3                      //
    pop r2                      //
    ret                         //

    .org 128
quicksort:
    push blink                  //
    push r2                     //
    push r3                     //
    push r4                     //
    mov r2, r0                  // r2 <- LO
    mov r4, r1                  // r4 <- HI
    cmp r1, r0                  //
    jle __quicksort_end         // if ((hi - lo) < 0) goto __quicksort_end//
    call partition              // r0 <- partition(lo, hi)//
    mov r3, r0                  // r3 <- PIVOT
    mov r0, r2                  //
    subi r1, r3, 1              //
    call quicksort              // quicksort(lo, p - 1)//
    addi r0, r3, 1              //
    mov r1, r4                  //
    call quicksort              // quicksort(p + 1, hi)//
__quicksort_end:
    pop r4                      //
    pop r3                      //
    pop r2                      //
    pop blink                   //
    ret                         // PC <- BLINK

    .org 256
__start:
    movi r0, 0                  // Initialize machine state on reset
    movi r1, 0
    movi r2, 0
    movi r3, 0
    movi r4, 0
    movi r5, 0
    movi r6, 0
    movi blink, 0
main:
__wait_for_next_job:
    wait                        // wait until queue_ready == 1
    movi r0, 0                   // r0 <- "Initial LO"
    movs r1, N                  // r1 <- "Initial HI (N)"
    call quicksort              // call quicksort(A, lo, hi)//
    emit                        //
    j __wait_for_next_job       // goto main

    .org 512
error:
    j error                     // TODO(shenry): figure out error behaviour.
