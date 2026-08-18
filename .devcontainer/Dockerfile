FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC

# OpenSSL 1.1 (uWebSockets e94b6e1 does not build against OpenSSL 3)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    git \
    wget \
    build-essential \
    cmake \
    make \
    gfortran \
    pkg-config \
    libuv1-dev \
    libssl-dev \
    zlib1g-dev \
    cppad \
    coinor-libipopt-dev \
    sudo \
    clang-format \
    clangd \
    && rm -rf /var/lib/apt/lists/*

# Udacity MPC expects this specific uWebSockets commit
RUN git clone https://github.com/uWebSockets/uWebSockets.git /tmp/uWebSockets \
    && cd /tmp/uWebSockets \
    && git checkout e94b6e1 \
    && mkdir build && cd build \
    && cmake .. && make && make install \
    && rm -rf /tmp/uWebSockets \
    && echo "/usr/local/lib" > /etc/ld.so.conf.d/local.conf \
    && ldconfig \
    && if [ -f /usr/local/lib/libuWS.so ]; then ln -sf /usr/local/lib/libuWS.so /usr/lib/libuWS.so; \
       elif [ -f /usr/lib64/libuWS.so ]; then ln -sf /usr/lib64/libuWS.so /usr/lib/libuWS.so; fi

WORKDIR /workspaces
