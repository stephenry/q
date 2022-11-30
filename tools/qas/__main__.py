##========================================================================== //
## Copyright (c) 2022, Stephen Henry
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##
## * Redistributions of source code must retain the above copyright notice, this
##   list of conditions and the following disclaimer.
##
## * Redistributions in binary form must reproduce the above copyright notice,
##   this list of conditions and the following disclaimer in the documentation
##   and/or other materials provided with the distribution.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
## LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
## CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
## SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
## INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
## CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.
##========================================================================== //

import argparse
import sys
import os
from lark import Lark, Transformer
from qlib.verilog import Module, Package

a = argparse.ArgumentParser()
a.add_argument('-s', type=argparse.FileType('r'), default=sys.stdin)
a.add_argument('-o', type=argparse.FileType('w'), default=sys.stdout)
a.add_argument('-p', '--pkg', type=argparse.FileType('w'))
args = a.parse_args()

parser = Lark.open('asm_grammar.lark', rel_to=__file__)


(modulename, suffix) = os.path.splitext(os.path.basename(args.o.name))
m = Module(modulename);
m.add_port('input', 'pc', 16)
m.add_port('output', 'inst', 16)
args.o.write(str(m))

if args.pkg:
    # Generate package
    p = Package(args.pkg.name)
    p.add_typedef_logic('rom_pc_t', 16);
    print(p)
    args.pkg.write(str(p))


sys.exit(0)


class Program:

    class CompileError(Exception):
        def __init__(self, msg):
            self.msg = msg

    def __init__(self):
        self.labels = {}
        self.instructions = []
        self.pc = 0
    def add_label(self, l):
        self.labels[l] = self.pc
    def add_instruction(self, i):
        self.instructions.append(i)
        self.pc += 1
    def print(self):
        for i, inst in enumerate(self.instructions):
            print(f'{i} {inst}')
    def compile(self):
        self.validate()

    def validate(self):
        for i, inst in enumerate(self.instructions):
            try:
                if isinstance(inst, Jump) or isinstance(inst, Call):
                    if inst.label not in self.labels.keys():
                        raise Program.CompileError(f'No known label: {inst.label}')
            except Exception as e:
                print(inst, " ", e.msg)

class Oprand:
    def __init__(self, s):
        self.s = s
    def __str__(self):
        return f'{self.s}'

class Jump: 
    def __init__(self, label, cc=None):
        self.label = label
        self.cc = cc
    def __str__(self):
        return f'j{self.cc} {self.label}'

class Call:
    def __init__(self, label):
        self.label = label
    def __str__(self):
        return f'call {self.label}'

class Push:
    def __init__(self, reg):
        self.reg = reg
    def __str__(self):
        return f'push {self.reg}'

class Pop:
    def __init__(self, reg):
        self.reg = reg
    def __str__(self):
        return f'pop {self.reg}'

class Ld:
    def __init__(self, lhs, rhs):
        self.lhs = lhs
        self.rhs = rhs
    def __str__(self):
        return f'ld {self.lhs}, [{self.rhs}]'

class St:
    def __init__(self, lhs, rhs):
        self.lhs = lhs
        self.rhs = rhs
    def __str__(self):
        return f'st [{self.lhs}], {self.rhs}'

class Mov:
    def __init__(self, lhs, rhs):
        self.lhs = lhs
        self.rhs = rhs
    def __str__(self):
        return f'mov {self.lhs}, {self.rhs}'

class Movs:
    def __init__(self, lhs, rhs):
        self.lhs = lhs
        self.rhs = rhs
    def __str__(self):
        return f'movs {self.lhs}, {self.rhs}'

class Ret:
    def __str__(self):
        return f'ret'

class Await:
    def __str__(self):
        return f'await'

class Emit:
    def __str__(self):
        return f'emit'

class MyTransformer(Transformer):
    def __init__(self, p):
        self.p = p
    def link(self, items):
        return items[0].value
    def label(self, items):
        self.p.add_label(items[0])
    def emit_imm(self, items):
        return Oprand(items[0])
    def emit_reg(self, items):
        return Oprand(items[0])
    def emit_blink(self, items):
        return Oprand('blink')
    def emit_ireg(self, items):
        return Oprand(items[0])
    def emit_special(self, items):
        return Oprand('N')
    def opcode_jump(self, items):
        return (items[0][0], items[0][1:])
    def emit_jump(self, items):
        label = items[1]
        cc = items[0][1]
        self.p.add_instruction(Jump(label, cc))
    def emit_call(self, items):
        label = items[0]
        self.p.add_instruction(Call(label))
    def emit_push(self, items):
        self.p.add_instruction(Push(items[0]))
    def emit_pop(self, items):
        self.p.add_instruction(Pop(items[0]))
    def emit_ld(self, items):
        (lhs, rhs) = items
        self.p.add_instruction(Ld(lhs, rhs))
    def emit_st(self, items):
        (lhs, rhs) = items
        self.p.add_instruction(St(lhs, rhs))
    def emit_mov(self, items):
        (lhs, rhs) = items
        self.p.add_instruction(Mov(lhs, rhs))
    def emit_movs(self, items):
        (lhs, rhs) = items
        self.p.add_instruction(Movs(lhs, rhs))
    def emit_ret(self, items):
        self.p.add_instruction(Ret())
    def emit_await(self, items):
        self.p.add_instruction(Await())
    def emit_emit(self, items):
        self.p.add_instruction(Emit())

tree = parser.parse(args.ucode.read())

p = Program()
t = MyTransformer(p)
t.transform(tree)

p.compile()

print(p.print())


print("Parsing complete")