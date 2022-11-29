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
