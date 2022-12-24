## ==================================================================== ##
## Copyright (c) 2022, Stephen Henry
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##
## * Redistributions of source code must retain the above copyright
##   notice, this list of conditions and the following disclaimer.
##
## * Redistributions in binary form must reproduce the above copyright
##   notice, this list of conditions and the following disclaimer in
##   the documentation and/or other materials provided with the
##   distribution.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
## FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
## COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
## INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
## (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
## SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
## HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
## STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
## OF THE POSSIBILITY OF SUCH DAMAGE.
## ==================================================================== ##

if (EXISTS $ENV{VERILATOR_ROOT})
  set(VERILATOR_ROOT $ENV{VERILATOR_ROOT})
  set(Verilator_EXE ${VERILATOR_ROOT}/bin/verilator)
  execute_process(COMMAND ${Verilator_EXE} --version
    OUTPUT_VARIABLE Verilator_VER)
  string(REGEX REPLACE "Verilator ([0-9]).([0-9]+).*" "\\1"
    VERILATOR_VERSION_MAJOR ${Verilator_VER})
  string(REGEX REPLACE "Verilator ([0-9]).([0-9]+).*" "\\2"
    VERILATOR_VERSION_MINOR ${Verilator_VER})
  set(VERILATOR_VERSION
    ${VERILATOR_VERSION_MAJOR}.${VERILATOR_VERSION_MINOR})
  message(STATUS "Found Verilator version: ${VERILATOR_VERSION}")

  macro (verilator_build vlib)
    add_library(${vlib} STATIC
      "${VERILATOR_ROOT}/include/verilated.cpp"
      "${VERILATOR_ROOT}/include/verilated_dpi.cpp"
      "${VERILATOR_ROOT}/include/verilated_save.cpp"
      "$<$<BOOL:${ENABLE_VCD}>:${VERILATOR_ROOT}/include/verilated_vcd_c.cpp>"
      )
    target_include_directories(${vlib}
      PRIVATE
      "${VERILATOR_ROOT}/include"
      "${VERILATOR_ROOT}/include/vltstd"
      )
  endmacro ()
else()
  # Configuration script expects and requires that the VERILATOR_ROOT
  # variable be set in the current environment. 
  message(WARNING [[
    "VERILATOR_ROOT has not been defined in the environment. "
    "Verilator not found! Simulation is not supported"
    ]])
endif ()
