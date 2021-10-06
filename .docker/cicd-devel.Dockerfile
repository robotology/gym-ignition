ARG from=ghcr.io/robotology/gym-ignition:codespaces-base
FROM ${from}

ARG runtime_user=root
USER $runtime_user

# Install bootstrapping tools system-wide
RUN pip install vcstool colcon-common-extensions &&\
    rm -r $HOME/.cache/pip

ARG CMAKE_BUILD_TYPE="Release"
ARG ignition_codename="fortress"
ARG ignition_default_channel="stable"

# Install Ignition Gazebo
ARG WS=/workspace
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-$ignition_default_channel $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/gazebo-$ignition_default_channel.list &&\
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - &&\
    sudo apt-get update &&\
    sudo mkdir $WS && sudo chown $runtime_user $WS &&\
    mkdir -p $WS/src &&\
    cd $WS/src &&\
    wget https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-$ignition_codename.yaml &&\
    vcs import < collection-$ignition_codename.yaml &&\
    sudo apt -y install --no-install-recommends \
        $(sort -u $(find . -iname 'packages-'$(lsb_release -cs)'.apt' -o -iname 'packages.apt') | grep -v -E "^libignition|^libsdformat" | tr '\n' ' ') &&\
    sudo rm -rf /var/lib/apt/lists/* &&\
    cd $WS &&\
    colcon graph &&\
    colcon build \
        --cmake-args \
            -GNinja \
            -DBUILD_TESTING:BOOL=OFF \
            -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
        --merge-install \
        &&\
    find build/ -type f -not -name 'CMakeCache.txt' -delete &&\
    echo "[[ -f $WS/install/setup.bash ]] && source $WS/install/setup.bash" | \
        sudo tee /etc/profile.d/08-colcon.sh
