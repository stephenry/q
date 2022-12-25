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

find_package(Python3 REQUIRED)
if (${Python3_FOUND})
    set(Q_VENV_ROOT ${CMAKE_BINARY_DIR}/.venv)
    message(STATUS "Generating Virtual Environment at: ${Q_VENV_ROOT}")
    message("${Python3_EXECUTABLE} -m venv ${Q_VENV_ROOT}")
    message("${Q_VENV_ROOT}/bin/pip3 install -r ${CMAKE_SOURCE_DIR}/requirements.tx")
    execute_process(COMMAND ${Python3_EXECUTABLE} -m venv  ${Q_VENV_ROOT})
    execute_process(COMMAND ${Q_VENV_ROOT}/bin/pip3 install
        -r ${CMAKE_SOURCE_DIR}/requirements.txt)
    set(Q_PYTHON3 ${Q_VENV_ROOT}/bin/python3)
else ()
    message(FATAL [[
        "Python3 not found; RTL compilation not supported."
    ]])
endif ()
