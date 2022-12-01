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

ireg = {}

def register(cls):
    ireg[cls.MNEMONIC] = cls
    return cls

class Instruction:
    pass

@register
class Jump(Instruction): 
    MNEMONIC = 'j'
    def __init__(self, label, cc=None):
        self.label = label
        self.cc = cc
    def __str__(self):
        cc = self.cc and self.cc or ""
        return f'j{cc} {self.label}'

@register
class Call(Instruction):
    MNEMONIC = 'call'
    def __init__(self, label):
        self.label = label
    def __str__(self):
        return f'call {self.label}'

@register
class Push(Instruction):
    MNEMONIC = 'push'
    def __init__(self, reg):
        self.reg = reg
    def __str__(self):
        return f'push {self.reg}'

@register
class Pop(Instruction):
    MNEMONIC = 'pop'
    def __init__(self, reg):
        self.reg = reg
    def __str__(self):
        return f'pop {self.reg}'

@register
class Ld(Instruction):
    MNEMONIC = 'ld'
    def __init__(self, dst, src):
        self.dst = dst
        self.src = src
    def __str__(self):
        return f'ld {self.dst}, [{self.src}]'

@register
class St(Instruction):
    MNEMONIC = 'st'
    def __init__(self, dst, src):
        self.dst = dst
        self.src = src
    def __str__(self):
        return f'st [{self.dst}], {self.src}'

@register
class Mov(Instruction):
    MNEMONIC = 'mov'
    def __init__(self, dst, src):
        self.dst = dst
        self.src = src
    def __str__(self):
        return f'mov {self.dst}, {self.src}'

@register
class Movs(Instruction):
    MNEMONIC = 'movs'
    def __init__(self, dst, src):
        self.dst = dst
        self.src = src
    def __str__(self):
        return f'movs {self.dst}, {self.src}'

@register
class Sub(Instruction):
    MNEMONIC = 'sub'
    def __init__(self, dst, lhs, rhs):
        self.dst = dst
        self.lhs = lhs
        self.rhs = rhs
    def __str__(self):
        return f'sub {self.dst}, {self.lhs}, {self.rhs}'

@register
class Add(Instruction):
    MNEMONIC = 'add'
    def __init__(self, dst, lhs, rhs):
        self.dst = dst
        self.lhs = lhs
        self.rhs = rhs
    def __str__(self):
        return f'add {self.dst}, {self.lhs}, {self.rhs}'

@register
class Ret(Instruction):
    MNEMONIC = 'ret'
    def __str__(self):
        return f'ret'

@register
class Wait(Instruction):
    MNEMONIC = 'wait'
    def __str__(self):
        return f'wait'

@register
class Emit(Instruction):
    MNEMONIC = 'emit'
    def __str__(self):
        return f'emit'

@register
class Call(Instruction):
    MNEMONIC = 'call'
    def __init__(self, label):
        self.label = label
    def __str__(self):
        return f'call {self.label}'

@register
class Cmp(Instruction):
    MNEMONIC = 'cmp'
    def __init__(self, lhs, rhs):
        self.lhs = lhs
        self.rhs = rhs
    def __str__(self):
        return f'cmp {self.lhs}, {self.rhs}'