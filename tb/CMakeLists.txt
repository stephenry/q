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
# Options:
set(ENABLE_VCD ON)

# ---------------------------------------------------------------------------- #
# Build sources:
include(rtl)

set(TARGET "sim")
include(tech)

# Model/Design configuration.
set(Q_ENGINES_N 16)
set(Q_CONTEXTS_N 32)
set(Q_CMP_KEY_BITS 32)
set(Q_CMP_VAL_BITS 32)
set(Q_CMP_MAX_N 1024)

# ---------------------------------------------------------------------------- #
# Configure sources
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cfg.h.in" cfg.h)

foreach (src_fn "${RTL_GENERATED_SOURCES}")
  cmake_path(GET src_fn STEM LAST_ONLY dst_fn)
  configure_file(${src_fn} ${dst_fn})
endforeach ()

# ---------------------------------------------------------------------------- #
# Verilate

# Build verilator support library
verilator_build(vlib)

set(VERILATOR_ARGS
  "-cc"
##  "-Wall"
  "--Mdir Vobj"
  "--build"
  "--top tb"
  )
if (ENABLE_VCD)
  list(APPEND VERILATOR_ARGS --trace)
endif ()

set(TB_SOURCES
  "${RTL_SOURCES}"
  "${TECH_SOURCES}"
  "${CMAKE_CURRENT_SOURCE_DIR}/tb.sv")

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
add_dependencies(verilate generate_ucode_rom)

set(VERILATOR_A "${CMAKE_CURRENT_BINARY_DIR}/Vobj/Vtb__ALL.a")

# ---------------------------------------------------------------------------- #
# Driver executable:
set(DRIVER_CPP
  "${CMAKE_CURRENT_SOURCE_DIR}/tb.cc"

  )

add_executable(driver ${DRIVER_CPP})
target_include_directories(driver PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${VERILATOR_ROOT}/include")
target_link_libraries(driver vlib ${VERILATOR_A})
add_dependencies(driver verilate)
