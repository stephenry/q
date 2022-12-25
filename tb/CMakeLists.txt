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
# Includes
#
include(sva/sva.cmake)

# ---------------------------------------------------------------------------- #
# Driver executable
#
add_executable(driver)

# ---------------------------------------------------------------------------- #
# Build verilator support library
#
verilator_build(vlib)

# ---------------------------------------------------------------------------- #
# Add subordinate, unit testbenches
#
add_subdirectory(tb_stk)

# ---------------------------------------------------------------------------- #
# Driver executable:
#
target_sources(driver
  PRIVATE
  "${CMAKE_SOURCE_DIR}/tb/test.cc"
  "${CMAKE_SOURCE_DIR}/tb/tb.cc"
  "${CMAKE_SOURCE_DIR}/tb/driver.cc"
  )

configure_file("${CMAKE_SOURCE_DIR}/tb/cfg_common.h.in" cfg_common.h)

target_include_directories(driver
  PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${VERILATOR_ROOT}/include"
  )

target_link_libraries(driver
  vlib
  stk
  )
