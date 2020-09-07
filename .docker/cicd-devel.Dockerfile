ARG from=diegoferigo/gym-ignition:base
FROM ${from}

ARG IGNITION_DEFAULT_CHANNEL="stable"
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-${IGNITION_DEFAULT_CHANNEL} `lsb_release -cs` main" > \
        /etc/apt/sources.list.d/gazebo-${IGNITION_DEFAULT_CHANNEL}.list &&\
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - &&\
    apt-get update &&\
    apt-get install -y --no-install-recommends \
        cmake \
        freeglut3-dev \
        libavcodec-dev \
        libavdevice-dev \
        libavformat-dev \
        libavutil-dev \
        libbenchmark-dev \
        libcurl4-openssl-dev \
        libfreeimage-dev \
        libgflags-dev \
        libglew-dev \
        libgts-dev \
        libjsoncpp-dev \
        libogre-1.9-dev \
        libogre-2.1-dev \
        libprotobuf-dev \
        libprotoc-dev \
        libqt5core5a \
        libsqlite3-dev \
        libswscale-dev \
        libtinyxml-dev \
        libtinyxml2-dev \
        libwebsockets-dev \
        libyaml-dev \
        libzip-dev \
        libzmq3-dev \
        pkg-config \
        protobuf-compiler \
        python \
        qml-module-qt-labs-folderlistmodel \
        qml-module-qt-labs-settings \
        qml-module-qtqml-models2 \
        qml-module-qtquick-controls \
        qml-module-qtquick-controls2 \
        qml-module-qtquick-dialogs \
        qml-module-qtquick-layouts \
        qml-module-qtquick2 \
        qtbase5-dev \
        qtdeclarative5-dev \
        qtquickcontrols2-5-dev \
        ruby \
        ruby-dev \
        ruby-ronn \
        swig \
        uuid-dev \
        # Additional
        libdart-collision-ode-dev \
        libdart-dev \
        libdart-external-ikfast-dev \
        libdart-external-odelcpsolver-dev \
        libdart-utils-urdf-dev \
        &&\
    rm -rf /var/lib/apt/lists/*

RUN pip3 install vcstool colcon-common-extensions &&\
    rm -r $HOME/.cache/pip

ARG CMAKE_BUILD_TYPE="Release"
ARG ignition_codename="citadel"

RUN mkdir -p /workspace/src &&\
    cd /workspace/src &&\
    wget https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-${ignition_codename}.yaml &&\
    vcs import < collection-${ignition_codename}.yaml &&\
    cd /workspace &&\
    colcon graph &&\
    colcon build \
        --cmake-args \
            -GNinja \
            -DBUILD_TESTING:BOOL=OFF \
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
        --merge-install \
        &&\
    echo "source /workspace/install/setup.bash" >> /etc/bash.bashrc

COPY entrypoint.sh /entrypoint.sh
COPY setup_virtualenv.sh /setup_virtualenv.sh
RUN chmod 755 /entrypoint.sh
RUN chmod 755 /setup_virtualenv.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
