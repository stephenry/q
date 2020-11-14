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

`ifndef QS_RTL_QS_SRT_PKG_VH
 `define QS_RTL_QS_SRT_PKG_VH

package qs_srt_pkg;

  // Program counter type.
  typedef logic [7:0] pc_t;

  // Machine reset vector.
  localparam pc_t RESET_VECTOR              = '0;

  // Condition codes
  typedef enum logic [1:0] { UNCOND = 2'b00,
                             EQ     = 2'b01,
                             GT     = 2'b10,
                             LE     = 2'b11
                             } cc_t;

  // Immediate field
  typedef logic [2:0] imm_t;

  // 'A'-type field definition.
  typedef logic [7:0] field_A_t;

  // Register synonyms:
  typedef enum logic [2:0] { R0    = 3'b000,
                             R1    = 3'b001,
                             R2    = 3'b010,
                             R3    = 3'b011,
                             R4    = 3'b100,
                             R5    = 3'b101,
                             R6    = 3'b110,
                             BLINK = 3'b111
                             } reg_t;

  typedef enum logic [2:0] { REG_N   = 3'b000
                             } reg_special_t;

  // Instruction oprands:
  typedef enum logic [3:0] { NOP   = 4'b0000,
                             JCC   = 4'b0001,
                             PP    = 4'b0010,
                             MEM   = 4'b0100,
                             MOV   = 4'b0110,
                             ARITH = 4'b0111,
                             CRET  = 4'b1100,
                             CMP   = 4'b1101,
                             CNTRL = 4'b1111
                            } opcode_t;

  // Packet instruction encoding
  typedef struct packed {
    opcode_t opcode;
    union packed {
      logic [11:0] raw;
      struct packed {
        logic [11:0] padding;
      } nop;
      struct packed {
        logic [1:0] padding;
        cc_t cc;
        field_A_t A;
      } jcc;
      struct  packed {
        logic is_pop;
        union packed {
          struct packed {
            reg_t dst;
            logic [7:0] padding0;
          } pop;
          struct packed {
            logic [7:0] padding0;
            reg_t src1;
          } push;
        } u;
      } pp;
      struct  packed {
        logic is_st;
        reg_t dst;
        union packed {
          struct packed {
            logic [3:0] padding0;
            logic       padding1;
            reg_t src1;
          } ld;
          struct packed {
            logic padding0;
            reg_t src0;
            logic padding1;
            reg_t src1;
          } st;
        } u;
      } mem;
      struct  packed {
        logic is_special;
        reg_t dst;
        logic [3:0] padding0;
        logic is_imm;
        union packed {
          reg_t src;
          reg_special_t special;
          imm_t imm;
        } u;
      } mov;
      struct  packed {
        logic is_sub;
        reg_t dst;
        logic wren;
        reg_t src0;
        logic is_imm;
        union packed {
          reg_t src1;
          imm_t imm;
        } u;
      } arith;
      struct  packed {
        logic is_ret;
        logic [2:0] padding;
        field_A_t a;
      } cret;
      struct  packed {
        logic is_emit;
        logic [10:0] padding;
      } cntrl;
    } u;
  } inst_t;

  // Decoded horizontal microcode.
  typedef struct packed {
    logic            is_emit;
    logic            is_await;
    logic            is_call;
    logic            is_ret;
    logic            is_load;
    logic            is_store;
    logic            is_jump;
    logic            is_push;
    logic            is_pop;

    //
    logic            dst_en;
    reg_t            dst;

    //
    logic            src0_is_blink;
    logic            src1_is_blink;

    //
    logic            src0_en;
    reg_t            src0;
    logic            src1_en;
    reg_t            src1;

    //
    logic            src0_is_zero;

    //
    logic            has_imm;
    imm_t            imm;

    //
    logic            has_special;
    reg_special_t    special;

    //
    logic            inv_src1;
    logic            cin;

    //
    logic            flag_en;

    //
    logic            invalid_inst;

    //
    cc_t             cc;
    field_A_t        target;

  } ucode_t;

  function automatic logic [2:0] R_field(inst_t inst);
    return inst[10:8];
  endfunction // R_field

  function automatic logic [2:0] S_field(inst_t inst);
    return inst[6:4];
  endfunction // S_field

  function automatic logic [2:0] SPECIAL_field(inst_t inst);
    return inst[2:0];
  endfunction // SPECIAL_field

  function automatic logic [2:0] U_field(inst_t inst);
    return inst[2:0];
  endfunction // U_field

  function automatic logic [2:0] I_field(inst_t inst);
    return inst[2:0];
  endfunction // I_field

  function automatic logic IMM_field(inst_t inst);
    return inst[3];
  endfunction // I_field

  function automatic logic W_field(inst_t inst);
    return inst[7];
  endfunction // W_field

  function automatic logic SEL_field(inst_t inst);
    return inst[11];
  endfunction // SEL_field

  function automatic logic [1:0] CC_field(inst_t inst);
    return inst[9:8];
  endfunction // CC_field

  function automatic logic [7:0] A_field(inst_t inst);
    return inst[7:0];
  endfunction // CC_field

  function automatic inst_t j (pc_t dest, cc_t cc = UNCOND); begin
    j = '0;
    //
    j.opcode    = JCC;
    j.u.jcc.cc  = cc;
    j.u.jcc.A   = field_A_t'(dest);
  end endfunction

  // Await instruction; awaits ready status of the currently selected
  // bank.
  function automatic inst_t await; begin
    await = '0;
    //
    await.opcode  = CNTRL;
  end endfunction

  // Emit instruction; notifies completion status of the currently
  // selected bank.
  function automatic inst_t emit; begin
    emit = '0;
    //
    emit.opcode           = CNTRL;
    emit.u.cntrl.is_emit  = 'b1;
  end endfunction

  // Call instruction
  function automatic inst_t call(pc_t dest); begin
    call = '0;
    //
    call.opcode    = CRET;
    call.u.cret.a  = field_A_t'(dest);
  end endfunction

  // Return instruction
  function automatic inst_t ret; begin
    ret                       = '0;
    //
    ret.opcode                = CRET;
    ret.u.cret.is_ret = 'b1;
  end endfunction

  // Push instruction
  function automatic inst_t push(reg_t r); begin
    push = '0;
    //
    push.opcode            = PP;
    push.u.pp.u.push.src1  = r;
  end endfunction

  // Pop instruction
  function automatic inst_t pop(reg_t r); begin
    // TODO: introduce one cycle hazard on pop.
    pop = '0;
    //
    pop.opcode          = PP;
    pop.u.pp.is_pop     = 'b1;
    pop.u.pp.u.pop.dst  = r;
  end endfunction

  // Move instruction; reg[dst] <- reg[src0]
  function automatic inst_t mov(reg_t dst, reg_t src0); begin
    mov = '0;
    //
    mov.opcode       = MOV;
    mov.u.mov.dst    = dst;
    mov.u.mov.u.src  = src0;
  end endfunction

  // Move Immediate instruction; reg[dst] <- ext(imm);
  function automatic inst_t movi(reg_t dst, imm_t imm); begin
    movi = '0;
    //
    movi.opcode        = MOV;
    movi.u.mov.dst     = dst;
    movi.u.mov.is_imm  = 'b1;
    movi.u.mov.u.imm   = imm;
  end endfunction

  // Move "Special" instruction reg[dst] <- special[src1]
  function automatic inst_t movs(reg_t dst, reg_special_t src1); begin
    movs = '0;
    //
    movs.opcode            = MOV;
    movs.u.mov.dst         = dst;
    movs.u.mov.is_special  = 'b1;
    movs.u.mov.u.special   = src1;
  end endfunction

  // Add instruction (immediate); reg[dst] = reg[src0] + imm
  function automatic inst_t addi(reg_t dst, reg_t src0, imm_t imm,
                 bit dst_en = 'b1); begin
    addi = '0;
    //
    addi.opcode          = ARITH;
    addi.u.arith.dst     = dst;
    addi.u.arith.src0    = src0;
    addi.u.arith.is_imm  = 'b1;
    addi.u.arith.u.imm   = imm;
    addi.u.arith.wren    = dst_en;
  end endfunction

  // Subtract instruction (immediate); reg[dst] = reg[src0] - imm
  function automatic inst_t subi(reg_t dst, reg_t src0, imm_t imm,
                 bit dst_en = 'b1); begin
    subi = '0;
    //
    subi.opcode          = ARITH;
    subi.u.arith.is_sub  = 'b1;
    subi.u.arith.dst     = dst;
    subi.u.arith.src0    = src0;
    subi.u.arith.is_imm  = 'b1;
    subi.u.arith.u.imm   = imm;
    subi.u.arith.wren    = dst_en;
  end endfunction

  // Subtract instructon: reg[dst] <- reg[src0] - reg[src1]
  function automatic inst_t sub(reg_t dst, reg_t src0, reg_t src1,
                bit dst_en = 'b1); begin
    sub = '0;
    //
    sub.opcode          = ARITH;
    sub.u.arith.is_sub  = 'b1;
    sub.u.arith.wren    = dst_en;
    sub.u.arith.dst     = dst;
    sub.u.arith.src0    = src0;
    sub.u.arith.u.src1  = src1;
  end endfunction

  // Load instruction: reg[dst] <- mem[src1]
  function automatic inst_t ld(reg_t dst, reg_t src1); begin
    ld = '0;
    //
    ld.opcode           = MEM;
    ld.u.mem.dst        = dst;
    ld.u.mem.u.ld.src1  = src1;
  end endfunction

  // Store instruction: mem[src0] <- reg[src1]
  function automatic inst_t st(reg_t src0, reg_t src1); begin
    st = '0;
    //
    st.opcode           = MEM;
    st.u.mem.is_st      = 'b1;
    st.u.mem.u.st.src0  = src0;
    st.u.mem.u.st.src1  = src1;
  end endfunction

  function automatic inst_t nop; begin
    nop = '0;
  end endfunction

  function automatic inst_t cmp(reg_t src0, reg_t src1); begin
    cmp                = '0;

    cmp.opcode         = CMP;
    cmp.u.arith.src0   = src0;
    cmp.u.arith.u.src1 = src1;
  end endfunction

endpackage // qs_srt_pkg

`endif //  `ifndef QS_RTL_QS_SRT_PKG_VH
