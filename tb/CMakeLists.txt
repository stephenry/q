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

# ---------------------------------------------------------------------------- #
# Verilate
#

# Build verilator support library
verilator_build(vlib)

# ---------------------------------------------------------------------------- #
# Add subordinate, unit testbenches
#
add_subdirectory(tb_stk)

# ---------------------------------------------------------------------------- #
# Driver executable:
#
set (TB_ROOT "${CMAKE_SOURCE_DIR}/tb")

configure_file("${TB_ROOT}/cfg.h.in" cfg.h)

set(DRIVER_CPP
  "${TB_ROOT}/tb_stk/tb_stk.cc"
  "${TB_ROOT}/test.cc"
  "${TB_ROOT}/tb.cc"
  "${TB_ROOT}/driver.cc"
  )

add_executable(driver ${DRIVER_CPP})
target_include_directories(driver PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${VERILATOR_ROOT}/include")
target_link_libraries(driver
  vlib

  # TODO: figure out a better way to import this dependency.
  "${CMAKE_CURRENT_BINARY_DIR}/tb_stk/Vobj/Vtb_stk__ALL.a"
  )


add_dependencies(driver verilate)
