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
# Build sources:
include(rtl)

set(TARGET "sim")
include(tech)

# Model/Design configuration.
set(Q_ENGINES_N 4)
set(Q_CONTEXTS_N 32)

# ---------------------------------------------------------------------------- #
# Configure sources
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cfg_pkg.vh.in" cfg_pkg.vh)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cfg.h.in" cfg.h)

set(VOUT_DIR "Vobj")
set(VTOP "tb_stk")

set(VERILATOR_ARGS
  "-cc"
#  "-Wall"
  "--Mdir ${VOUT_DIR}"
  "--build"
  "--top ${VTOP}"
  )
if (ENABLE_VCD)
  list(APPEND VERILATOR_ARGS --trace)
endif ()

set(TB_SOURCES
  "${COMMON_SOURCES}"
  "${STK_SOURCES}"
  "${TECH_SOURCES}"
  "${CMAKE_CURRENT_SOURCE_DIR}/tb_stk.sv")

set(VERILATOR_INCLUDES "-I${CMAKE_CURRENT_BINARY_DIR}")
foreach (inc_fn ${RTL_INCLUDE_PATHS})
  list(APPEND VERILATOR_INCLUDES "-I${inc_fn}")
endforeach ()
foreach (inc_fn ${TECH_INCLUDE_PATHS})
  list(APPEND VERILATOR_INCLUDES "-I${inc_fn}")
endforeach ()

set(VERILATOR_COMMAND_LIST
  "${VERILATOR_ARGS}"
  "${VERILATOR_INCLUDES}"
  "${TB_SOURCES}")

string(REGEX REPLACE ";" "\n" FILELIST "${VERILATOR_COMMAND_LIST}")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/vc.f "${FILELIST}")

add_custom_target(verilate
  COMMAND ${Verilator_EXE} -f ${CMAKE_CURRENT_BINARY_DIR}/vc.f
  COMMENT "Verilating...")

add_dependencies(driver verilate)
target_link_libraries(driver
  "${CMAKE_CURRENT_BINARY_DIR}/${VOUT_DIR}/V${VTOP}__ALL.a")
