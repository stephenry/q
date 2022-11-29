;;========================================================================== ;;
;; Copyright (c) 2022, Stephen Henry
;; All rights reserved.
;;
;; Redistribution and use in source and binary forms, with or without
;; modification, are permitted provided that the following conditions are met:
;;
;; * Redistributions of source code must retain the above copyright notice, this
;;   list of conditions and the following disclaimer.
;;
;; * Redistributions in binary form must reproduce the above copyright notice,
;;   this list of conditions and the following disclaimer in the documentation
;;   and/or other materials provided with the distribution.
;;
;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;; POSSIBILITY OF SUCH DAMAGE.
;;========================================================================== ;;

reset:
    j __start                   ; reset vector, jump to main subroutine.
partition:
    push R2                     ;
    push R3                     ;
    push R4                     ;
    push R5                     ;
    push R6                     ;
    ld R2, [R1]                 ; pivot <- A[hi];
    mov R3, R0                  ; i <- lo
    mov R4, R0                  ; j <- lo   
__loop_start:
    sub 0, R1, R4               ;
    JEQ __end                   ; if (j == hi) goto __end
    ld R5, [R4]                 ; R5 <- A[j]
    sub.f 0, R5, R2             ;
    jgt __end_loop              ; if ((A[j] - pivot) > 0) goto __end_of_loop
    ld R6, [R3]                 ; swap A[i] with A[j]
    st [R3], R5                 ;
    st [R4], R6                 ;
    addi R3, R3, 1              ; i <- i + 1
__end_loop:
    addi R4, R4, 1              ; j <- j + 1
    j __loop_start              ;
__end:
    ld R0, [R3]                 ;
    ld R1, [R4]                 ;
    st [R3], R1                 ;
    st [R4], R0                 ;
    mov R0, R3                  ; ret <- pivot
    pop R6                      ;
    pop R5                      ;
    pop R4                      ;
    pop R3                      ;
    pop R2                      ;
    ret                         ;
quicksort:
    push BLINK                  ;
    push R2                     ;
    push R3                     ;
    push R4                     ;
    mov R2, R0                  ; R2 <- LO
    mov R4, R1                  ; R4 <- HI
    sub.f 0, R1, R0             ;
    jle __quicksort_end         ; if ((hi - lo) < 0) goto __end;
    call PARTITION              ; R0 <- partition(lo, hi);
    mov R3, R0                  ; R3 <- PIVOT
    mov R0, R2                  ;
    subi R1, R3, 1              ;
    call QUICKSORT              ; quicksort(lo, p - 1);
    addi R0, R3, 1              ;
    mov R1, R4                  ;
    call QUICKSORT              ; quicksort(p + 1, hi);
__quicksort_end:
    pop R4                      ;
    pop R3                      ;
    pop R2                      ;
    pop BLINK                   ;
    ret                         ; PC <- BLINK
__start:
    await                       ; wait until queue_ready == 1
    movi R0, 0                  ; R0 <- "Initial LO"
    movs R1, N                  ; R1 <- "Initial HI (N)"
    call __qs                   ; call quicksort(A, lo, hi);
    emit                        ;
    J __main                    ; goto __main

;; end