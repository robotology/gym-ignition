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
        clang-7 \
        clang-8 \
        make \
        cmake \
        ninja-build \
        valgrind \
        ccache \
        libgflags-dev \
        python3-pip \
        python3.6 \
        python3.7 \
        libpython3.6-dev \
        libpython3.7-dev \
        &&\
    rm -rf /var/lib/apt/lists/*

# Enable CCACHE by default
ENV PATH=/usr/lib/ccache:${PATH}
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

# Install ign-gazebo from my fork
RUN cd /tmp &&\
    git clone --depth 1 https://github.com/diegoferigo/ign-gazebo  &&\
    cd ign-gazebo && mkdir -p build && cd build &&\
    cmake \
        -DCMAKE_BUILD_TYPE="Release" \
        -DBUILD_TESTING:BOOL=OFF \
        -DCMAKE_INSTALL_PREFIX="/usr" \
        .. &&\
    cmake --build . --target install -- -j8 &&\
    rm -r /tmp/ign-gazebo

# Install idyntree
RUN apt-get update &&\
    apt-get install -y --no-install-recommends python3-numpy libxml2-dev coinor-libipopt-dev &&\
    rm -rf /var/lib/apt/lists/* &&\
    git clone --depth 1 -b feature/numpy-and-ik https://github.com/diegoferigo/idyntree /tmp/idyntree &&\
    mkdir -p /tmp/idyntree/build && cd /tmp/idyntree/build &&\
    cmake .. \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DIDYNTREE_USES_PYTHON=True \
        -DPython_ADDITIONAL_VERSIONS=3.6 \
        -DIDYNTREE_USES_PYTHON_VERSION=3.6 \
        -DPYTHON_NUMPY_INCLUDE_DIR:PATH=/usr/lib/python3/dist-packages/numpy/core/include \
        &&\
    cmake --build . --target install &&\
    rm -r /tmp/idyntree
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
ENV PYTHONPATH=${PYTHONPATH}:/usr/local/lib/python3/dist-packages

# Setup gym-ignition
ARG install_prefix="/usr/local"
ENV IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${install_prefix}/lib/gympp/plugins"
ENV IGN_GAZEBO_RESOURCE_PATH="${install_prefix}/share/gympp/gazebo/worlds:${install_prefix}/share/gympp/gazebo/models"
ENV SDF_PATH="${install_prefix}/share/gympp/gazebo/models"
ENV PYTHONPATH="${PYTHONPATH}:${install_prefix}/lib/gympp/bindings"

# Prepare virtualenv variables
ENV VIRTUAL_ENV=/venv
ENV PATH=${VIRTUAL_ENV}/bin:${PATH}

COPY entrypoint.sh /entrypoint.sh
COPY setup_virtualenv.sh /setup_virtualenv.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
