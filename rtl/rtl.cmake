##========================================================================== //
## Copyright (c) 2020, Stephen Henry
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

set(SRC_ROOT "${CMAKE_SOURCE_DIR}/rtl")
set(BIN_ROOT "${CMAKE_BINARY_DIR}/rtl")

set(RTL_SOURCES
    "${SRC_ROOT}/common/rf.sv"
    "${SRC_ROOT}/common/mask.sv"
    "${SRC_ROOT}/common/lzd.sv"
    "${SRC_ROOT}/common/pri.sv"
    "${SRC_ROOT}/common/cla.sv"
    "${SRC_ROOT}/common/cmp.sv"
    "${SRC_ROOT}/common/dec.sv"
    "${SRC_ROOT}/common/mux.sv"
    "${SRC_ROOT}/eng/eng_pipe_ca.sv"
    "${BIN_ROOT}/eng/eng_pipe_xa_rom.sv"
    "${BIN_ROOT}/eng/eng_pipe_xa_dec.sv"
    "${SRC_ROOT}/eng/eng_pipe_xa.sv"   
    "${SRC_ROOT}/eng/eng_pipe_fa.sv"
    "${SRC_ROOT}/eng/eng_pipe.sv"
    "${SRC_ROOT}/eng/eng.sv"
    "${SRC_ROOT}/q_engs.sv"
    "${SRC_ROOT}/q.sv")

set(RTL_GENERATED_SOURCES
    "${SRC_ROOT}/cfg_pkg.vh.in")

set(RTL_INCLUDE_PATHS
    "${BIN_ROOT}"
    "${SRC_ROOT}")