ARG VARIANT="jammy"
FROM mcr.microsoft.com/vscode/devcontainers/base:0-${VARIANT}
RUN apt-get update \
  && DEBIAN_FRONTEND=noninterative \
    apt-get install --no-install-recommends -y \
      autoconf \
      bc \
      bison \
      build-essential \
      flex \
      libfl-dev \
      gdb \
      cmake \
      python3-pip \
      python3-venv \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*


ARG VERILATOR_VER="5.002"

WORKDIR /tools
RUN wget https://github.com/verilator/verilator/archive/refs/tags/v${VERILATOR_VER}.tar.gz \
  && tar -zxf v${VERILATOR_VER}.tar.gz \
  && cd verilator-${VERILATOR_VER} \
  && autoconf \
  && ./configure \
  && make \
  && cd .. \
  && rm -rf v${VERILATOR_VER}.tar.gz

# Required for the build system.
ENV VERILATOR_ROOT="/tools/verilator-${VERILATOR_VER}"

# Required for the linter.
ENV PATH="${VERILATOR_ROOT}/bin:$PATH"

ARG ESPRESSO_VER=2.4
WORKDIR /tools
RUN wget https://github.com/chipsalliance/espresso/archive/refs/tags/v${ESPRESSO_VER}.tar.gz \
  && tar -xvf v${ESPRESSO_VER}.tar.gz \
  && cd espresso-${ESPRESSO_VER} \
  && mkdir build \
  && cd build \
  && cmake .. -DCMAKE_INSTALL_PREFIX=/tools/espresso-${ESPRESSO_VER} \
  && make \
  && make install \
  && cd /tools \
  && rm -rf v${ESPRESSO_VER}.tar.gz

ENV ESPRESSO_ROOT="/tools/espresso-${ESPRESSO_VER}"
ENV PATH="${ESPRESSO_ROOT}/bin:$PATH"
