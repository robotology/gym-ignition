ARG from=ubuntu:focal
FROM ${from}

SHELL ["/bin/bash", "-c"]

# Setup locales and timezone
ARG TZ=Europe/Rome
ARG DEBIAN_FRONTEND=noninteractive
RUN rm -f /etc/localtime &&\
    ln -s /usr/share/zoneinfo/"${TZ}" /etc/localtime &&\
    apt-get update &&\
    apt-get install -y --no-install-recommends locales locales-all tzdata &&\
    rm -rf /var/lib/apt/lists/*

# Install tools and toolchain
RUN apt-get update &&\
    apt-get install -y --no-install-recommends \
        wget \
        software-properties-common \
        apt-transport-https \
        apt-utils \
        gnupg2 \
        nano \
        rename \
        source-highlight \
        &&\
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | apt-key add - && \
    apt-add-repository "deb https://apt.kitware.com/ubuntu/ `lsb_release -cs` main" &&\
    add-apt-repository ppa:deadsnakes/ppa &&\
    wget -nv -O - http://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - &&\
    apt-add-repository -y "deb http://apt.llvm.org/`lsb_release -cs`/ llvm-toolchain-`lsb_release -cs`-10 main" &&\
    apt-get update &&\
    apt-get install -y --no-install-recommends \
        build-essential \
        clang-10 \
        git \
        cmake \
        cmake-curses-gui \
        ninja-build \
        valgrind \
        libgflags-dev \
        python3-pip \
        python3-wheel \
        python3.8 \
        python3.8-dev \
        libpython3.8-dev \
        virtualenv \
        swig \
        &&\
    rm -rf /var/lib/apt/lists/*

# Update git (required by actions/checkout)
RUN add-apt-repository ppa:git-core/ppa &&\
    apt-get update &&\
    apt-get install -y --no-install-recommends git &&\
    rm -rf /var/lib/apt/lists/*

# Common virtualenv properties
ENV VIRTUAL_ENV=/venv
ENV PATH=$VIRTUAL_ENV/bin:$PATH

CMD ["bash"]
