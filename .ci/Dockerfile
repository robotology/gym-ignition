ARG from=ubuntu:bionic
FROM ${from}

# Install tools and toolchain
RUN apt-get update &&\
    apt-get install -y --no-install-recommends \
        wget \
        software-properties-common \
        apt-transport-https \
        apt-utils \
        gnupg2 \
        nano \
        &&\
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | apt-key add - && \
    apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main' &&\
    apt-get update &&\
    apt-get install -y --no-install-recommends \
        git \
        g++ \
        g++-8 \
        clang \
        make \
        cmake \
        ninja-build \
        ccache \
        libgflags-dev \
        python3-pip \
        &&\
    rm -rf /var/lib/apt/lists/*

# Enable CCACHE by default
ENV PATH=/usr/lib/ccache:$PATH
RUN mkdir "$HOME/.ccache" &&\
    echo "max_size = 2.0G" > "$HOME/.ccache/ccache.conf"

# Install swig
RUN apt-get update &&\
    apt-get install -y --no-install-recommends \
        autotools-dev \
        automake \
        bison \
        libpcre3-dev &&\
    rm -rf /var/lib/apt/lists/* &&\
    cd /tmp/ &&\
    git clone --depth 1 -b rel-4.0.0 https://github.com/swig/swig.git &&\
    cd swig &&\
    sh autogen.sh &&\
    ./configure &&\
    make -j8 &&\
    make install &&\
    rm -r /tmp/swig

# Install ignition robotics
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" \
        > /etc/apt/sources.list.d/gazebo-stable.list &&\
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-prerelease `lsb_release -cs` main" \
        > /etc/apt/sources.list.d/gazebo-prerelease.list &&\
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" \
        > /etc/apt/sources.list.d/gazebo-prerelease.list &&\
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - &&\
    apt-get update &&\
    apt-get install -y --no-install-recommends \
        libignition-cmake2-dev \
        libignition-plugin-dev \
        libignition-math6-dev \
        libignition-common3-dev \
        libignition-transport7-dev \
        libignition-msgs4-dev \
        libignition-tools-dev \
        libignition-fuel-tools3-dev \
        libsdformat8-dev \
        libignition-physics-dev \
        libignition-rendering-dev \
        libignition-sensors2-dev \
        libignition-gui2-dev \
        &&\
    rm -rf /var/lib/apt/lists/*

# Install ign-gazebo
RUN cd /tmp &&\
    git clone --depth 1 https://github.com/diegoferigo/ign-gazebo  &&\
    cd ign-gazebo && mkdir -p build && cd build &&\
    cmake \
        -DCMAKE_BUILD_TYPE="Release" \
        -DBUILD_TESTING:BOOL=OFF \
        .. &&\
    cmake --build . --target install -- -j8 &&\
    rm -r /tmp/ign-gazebo

# Install python packages in a virtualenv and enable it by default
ENV VIRTUAL_ENV=/ve
ENV PATH=${VIRTUAL_ENV}/bin:${PATH}
RUN pip3 install virtualenv &&\
    virtualenv ${VIRTUAL_ENV} &&\
    ${VIRTUAL_ENV}/bin/pip3 install \
        gym \
        numpy \
        pytest
    
# Setup gym-ignition
ARG install_prefix="/usr/local"
ARG INSTALL_PREFIX=${install_prefix}
ENV IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${INSTALL_PREFIX}/lib/gympp/plugins"
ENV IGN_GAZEBO_RESOURCE_PATH="${INSTALL_PREFIX}/share/gympp/gazebo/worlds:${INSTALL_PREFIX}/share/gympp/gazebo/models"
ENV SDF_PATH="${INSTALL_PREFIX}/share/gympp/gazebo/models"
ENV PYTHONPATH="${PYTHONPATH}:${INSTALL_PREFIX}/lib/gympp/bindings"
