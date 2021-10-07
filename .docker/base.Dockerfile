# This image could be built either from a plain ubuntu:focal image
# or a customized mcr.microsoft.com/vscode/devcontainers/base:focal
ARG from=ubuntu:focal
FROM ${from}

# Source /etc/profile.d/* scripts before each command.
# This trick introduces a single file that is read by all users during build time.
SHELL ["/bin/bash", "-l", "-c"]
CMD ["/bin/bash", "-l"]

# Install sudo and run all the commands as the runtime user,
# assuming it has a password-less configuration
RUN apt-get update &&\
    apt-get install -y --no-install-recommends sudo &&\
    rm -rf /var/lib/apt/lists/*
ARG runtime_user=root
USER $runtime_user

# Setup locales and timezone
ARG TZ=Europe/Rome
ARG DEBIAN_FRONTEND=noninteractive
RUN sudo rm -f /etc/localtime &&\
    sudo ln -s /usr/share/zoneinfo/"$TZ" /etc/localtime &&\
    sudo apt-get update &&\
    sudo apt-get install -y --no-install-recommends locales locales-all tzdata &&\
    sudo rm -rf /var/lib/apt/lists/*

# Install tools and toolchain
RUN sudo apt-get update &&\
    sudo apt-get install -y --no-install-recommends \
        wget \
        software-properties-common \
        apt-transport-https \
        apt-utils \
        gnupg2 \
        &&\
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add - && \
    sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ `lsb_release -cs` main" &&\
    sudo add-apt-repository ppa:deadsnakes/ppa &&\
    wget -nv -O - http://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add - &&\
    sudo apt-add-repository -y "deb http://apt.llvm.org/`lsb_release -cs`/ llvm-toolchain-`lsb_release -cs`-10 main" &&\
    sudo apt-get update &&\
    sudo apt-get install -y --no-install-recommends \
        build-essential \
        clang \
        git \
        cmake \
        cmake-curses-gui \
        ccache \
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
        colordiff \
        cppcheck \
        doxygen \
        graphviz \
        gdb \
        valgrind \
        iputils-ping \
        tree \
        nano \
        rename \
        source-highlight \
        &&\
    sudo rm -rf /var/lib/apt/lists/*

# Create the virtualenv and enable it for all "bash -i" shells
ARG virtual_env_dir=/opt/venv
RUN sudo chmod a+rw /opt &&\
    virtualenv -p $(which python3) $virtual_env_dir &&\
    source $virtual_env_dir/bin/activate &&\
    python -m pip install --upgrade pip &&\
    rm -r $HOME/.cache/pip &&\
    echo "[[ -z \"\$VIRTUAL_ENV\" ]] && source $virtual_env_dir/bin/activate" | \
        sudo tee -a /etc/profile.d/07-virtualenv.sh

# Install iDynTree. This is needed to export the CMake targets that will be used
# to build ScenarIO from source in Developer mode. In User mode, this package
# is ignored since a new clean PEP517 environment will be temporarily created.
RUN sudo apt-get update &&\
    sudo apt-get install -y --no-install-recommends \
        libxml2-dev coinor-libipopt-dev libeigen3-dev libassimp-dev swig &&\
    sudo rm -rf /var/lib/apt/lists/* &&\
    pip install idyntree &&\
    rm -rf $HOME/.cache/pip
ENV CMAKE_PREFIX_PATH=$virtual_env_dir/lib/python3.8/site-packages/idyntree:$CMAKE_PREFIX_PATH

# Add the GitHub Codespaces helpers
COPY codespaces_helpers.sh /etc/profile.d/99-codespaces_helpers.sh
