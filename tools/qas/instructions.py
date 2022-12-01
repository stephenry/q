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

class Instruction:
    pass

class Jump(Instruction): 
    def __init__(self, label, cc=None):
        self.label = label
        self.cc = cc

    def __str__(self):
        if self.cc:
            return f'j{self.cc} {self.label}'
        else:
            return f'j {self.label}'

class Call(Instruction):
    def __init__(self, label):
        self.label = label
    def __str__(self):
        return f'call {self.label}'

class Push(Instruction):
    def __init__(self, reg):
        self.reg = reg
    def __str__(self):
        return f'push {self.reg}'

class Pop(Instruction):
    def __init__(self, reg):
        self.reg = reg
    def __str__(self):
        return f'pop {self.reg}'

class Ld(Instruction):
    def __init__(self, dst, src):
        self.dst = dst
        self.src = src
    def __str__(self):
        return f'ld {self.dst}, [{self.src}]'

class St(Instruction):
    def __init__(self, dst, src):
        self.dst = dst
        self.src = src
    def __str__(self):
        return f'st [{self.dst}], {self.src}'

class Mov(Instruction):
    def __init__(self, dst, src):
        self.dst = dst
        self.src = src
    def __str__(self):
        return f'mov {self.dst}, {self.src}'

class Movs(Instruction):
    def __init__(self, dst, src):
        self.dst = dst
        self.src = src
    def __str__(self):
        return f'movs {self.dst}, {self.src}'

class Sub(Instruction):
    def __init__(self, dst, lhs, rhs):
        self.dst = dst
        self.lhs = lhs
        self.rhs = rhs
    def __str__(self):
        return f'sub {self.dst}, {self.lhs}, {self.rhs}'

class Add(Instruction):
    def __init__(self, dst, lhs, rhs):
        self.dst = dst
        self.lhs = lhs
        self.rhs = rhs
    def __str__(self):
        return f'add {self.dst}, {self.lhs}, {self.rhs}'

class Ret(Instruction):
    def __str__(self):
        return f'ret'

class Wait(Instruction):
    def __str__(self):
        return f'wait'

class Emit(Instruction):
    def __str__(self):
        return f'emit'

class Call(Instruction):
    def __init__(self, label):
        self.label = label
    def __str__(self):
        return f'call {self.label}'

class Cmp(Instruction):
    def __init__(self, lhs, rhs):
        self.lhs = lhs
        self.rhs = rhs
    def __str__(self):
        return f'cmp {self.lhs}, {self.rhs}'