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
from lark import Lark, Transformer, Visitor
from qlib.verilog import Module, Package
import cfg
from .instructions import *

a = argparse.ArgumentParser()
a.add_argument('-s', type=argparse.FileType('r'), default=sys.stdin)
a.add_argument('-o', type=argparse.FileType('w'), default=sys.stdout)
a.add_argument('-p', '--pkg', type=argparse.FileType('w'))
args = a.parse_args()


class AssemblyTransformer(Transformer):
    def __init__(self):
        self.pc = 0
        self.labels = {}
        self.insts = []

    def print(self):
        for inst in self.insts:
            print(*inst)

    def directive(self, args):
        (op, *arg) = args
        if op == '.org':
            self.pc = int(arg[0])
        else:
            pass

    def label(self, args):
        (label,) = args
        self._add_label(label)

    def inst_operand(self, args):
        constructors = {
            'push': Push,
            'pop': Pop,
            'ld': Ld,
            'st': St,
            'mov': Mov,
            'add': Add,
            'sub': Sub,
            'cmp': Cmp
        }
        (opcode, oprands) = args
        if opcode in constructors:
            self._add_inst(constructors[opcode](*oprands.children))

    def inst_to_link(self, args):
        (opcode, label) = args
        if opcode.startswith('j'):
            def cc():
                if len(opcode) == 1:
                    return None
                return opcode[1:]
            self._add_inst(Jump(label, cc()))
        elif opcode == 'call':
            self._add_inst(Call(label))
        else:
            pass

    def inst_no_operand(self, opcode):
        inst_constructor = {
            'ret': Ret,
            'wait': Wait,
            'emit': Emit
        }
        (opcode,) = opcode
        if opcode in inst_constructor:
            self._add_inst(inst_constructor[opcode]())

    def _add_inst(self, inst : Instruction):
        self.insts.append((self.pc, inst))
        self.pc += 1

    def _add_label(self, label):
        self.labels[label[:-1]] = self.pc

import os
from pathlib import Path

at = AssemblyTransformer()
with open(Path(__file__).parent.absolute() / 'grammar.lark', 'r') as f:
    asparser = Lark(f.read(), parser='lalr', transformer=at)
t = asparser.parse(args.s.read())
at.print()
#print(t.pretty())

sys.exit(0)


(modulename, suffix) = os.path.splitext(os.path.basename(args.o.name))
m = Module(modulename);
m.add_port('input', 'pc', 16)
m.add_port('output', 'inst', 16)
args.o.write(str(m))

if args.pkg:
    # Generate package
    p = Package(args.pkg.name)
    p.add_typedef_logic('rom_pc_t', 16);
    args.pkg.write(str(p))



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

    def set_pc(self, pc):
        self.pc = pc

    def add_instruction(self, i : Instruction):
        self.instructions.append((self.pc, i))
        self.pc += 1

    def print(self):
        for pc, inst in self.instructions:
            print(f'{pc} {inst}')
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

class AssemblerVisitor(Visitor):
    def __init__(self, p : Program):
        self.p = p
    def org(self, t):
        (pos,) = t.children
        self.p.set_pc(int(pos))
    def label(self, t):
        (label,) = t.children
        self.p.add_label(label)
    def jump(self, t):
        (insn, label) = t.children
        cc = None 
        if (len(insn) > 1): cc = insn[1:]
        self.p.add_instruction(Jump(label, cc))
    def push(self, t):
        (dst,) = t.children
        self.p.add_instruction(Push(dst))
    def pop(self, t):
        (dst,) = t.children
        self.p.add_instruction(Pop(dst))
    def ld(self, t):
        (dst, src) = t.children
        self.p.add_instruction(Ld(dst, src))
    def st(self, t):
        (dst, src) = t.children
        self.p.add_instruction(St(dst, src))
    def mov(self, t):
        (dst, src) = t.children
        self.p.add_instruction(Mov(dst, src))
    def movs(self, t):
        (dst, src) = t.children
        self.p.add_instruction(Mov(dst,src))
    def sub(self, t):
        (dst, lhs, rhs) = t.children
        self.p.add_instruction(Sub(dst, lhs, rhs))
    def add(self, t):
        (dst, lhs, rhs) = t.children
        self.p.add_instruction(Add(dst, lhs, rhs))
    def emit(self, t):
        self.p.add_instruction(Emit())
    def wait(self, t):
        self.p.add_instruction(Wait())
    def ret(self, t):
        self.p.add_instruction(Ret())
    def call(self, t):
        (label,) = t.children
        self.p.add_instruction(Call(label))
    def cmp(self, t):
        (rhs, lhs) = t.children
        self.p.add_instruction(Cmp(rhs, lhs))

tree = parser.parse(args.s.read())

p = Program()
v = AssemblerVisitor(p)
v.visit(tree)

p.print()

print("Parsing complete")