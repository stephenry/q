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

import os
import re

COPYRIGHT="""
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
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// ======================================================================== //
"""

def align_to(s, idx):
    align_n = idx - len(s)
    if align_n > 0:
        s += ' ' * align_n
    return s

class Module:
    HDR_PREFIX="""
module {} (
"""

    HDR_SUFFIX="""
);
"""

    SUFFIX = """
endmodule : {}
"""

    def __init__(self, module_name):
        self.module_name = module_name
        self.ports = []
    def add_port(self, dir, name, width):
        self.ports.append((dir, name, width))
    def __str__(self):
        render = ''
        render += COPYRIGHT
        render += Module.HDR_PREFIX.format(self.module_name)
        if self.ports:
            render += self._render_portlist()
        render += Module.HDR_SUFFIX
        render += Module.SUFFIX.format(self.module_name)
        return render
    def _render_portlist(self):
        portlist = ''
        def render_port(port, **args): 
            l = '';
            if 'prefix' in args:
                l += args['prefix']
            (direction, name, width) = port
            l += (direction == 'input') and 'input' or 'output'
            l += f' wire logic [{width - 1}:0]'
            l = align_to(l, 50)
            l += (direction == 'input') and 'i_' or 'o_'
            l += name
            return l
        portlist += render_port(self.ports[0], prefix='  ')
        for port in self.ports[1:]:
            portlist += '\n'
            portlist += render_port(port, prefix=', ')
        return portlist

class Package:
    PREFIX = '''
package {};
'''
    SUFFIX = '''
endpackage : {}
'''
    def __init__(self, package_path):
        (self.package_name,_) = os.path.splitext(os.path.basename(package_path))
        self.guard = self._generate_guard(package_path)
        self.localparams = []
        self.typedefs_logic = []

    def __str__(self):
        render = ''
        render += COPYRIGHT
        render += f'`ifndef {self.guard}\n'
        render += f'`define {self.guard}\n'
        render += Package.PREFIX.format(self.package_name)
        render += self._render_typedefs_logic()
        render += self._render_localparams()
        render += Package.SUFFIX.format(self.package_name)
        render += '`endif\n'
        return render

    def add_localparm(self, type, name, value):
        self.localparams.append((type, name, value))

    def add_typedef_logic(self, name, width):
        self.typedefs_logic.append((name, width));

    def _generate_guard(self, path):
        l = re.split(f'\.|{os.sep}', path.upper())
        return '_'.join(l[1:])

    def _render_typedefs_logic(self):
        r = ''
        def render_typedef_logic(name, width):
            return f'typedef logic [{width - 1}:0] {name};\n'
        for typedef_logic in self.typedefs_logic:
            r += render_typedef_logic(*typedef_logic)
        return r

    def _render_localparams(self):
        r = ''
        def render_localparam(type, name, value):
            r += f'localparam {type} {name} = {value};\n'
        for localparam in self.localparams:
            render_localparam(*localparam)
        return r