#!/bin/sh
set -e
set -u

# Set the installation prefix
INSTALL_PREFIX="${TRAVIS_INSTALL_PREFIX:-$HOME/local}"

cd $TRAVIS_BUILD_DIR
mkdir -p build && cd build

if [ "$TRAVIS_CMAKE_GENERATOR" = "Visual Studio 15 2017" ] ; then
    # Build and install gym-ignition
    cmake -G"$TRAVIS_CMAKE_GENERATOR" \
          -A"${TRAVIS_CMAKE_ARCHITECTURE}" \
          -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
          ..
    cmake --build . --config $TRAVIS_BUILD_TYPE
    cmake --build . --target INSTALL
elif [ "$TRAVIS_CMAKE_GENERATOR" = "Xcode" ] ; then
    # Build and install gym-ignition
    cmake -G"$TRAVIS_CMAKE_GENERATOR" \
          -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
          ..
    cmake --build . --config $TRAVIS_BUILD_TYPE
    cmake --build . --target install
else
    # Build and install gym-ignition
    cmake -G"$TRAVIS_CMAKE_GENERATOR" \
          -DCMAKE_BUILD_TYPE=$TRAVIS_BUILD_TYPE \
          -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
          ..
    cmake --build .
    cmake --build . --target install
fi

# Setup the environment
case $TRAVIS_OS_NAME in
    "linux")
        # gympp env vars already configured in the docker image
        export PYTHONPATH=$TRAVIS_BUILD_DIR/build/bindings:${PYTHONPATH:-}
    ;;
    "osx")
        export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$INSTALL_PREFIX/lib/gympp/plugins"
        export IGN_GAZEBO_RESOURCE_PATH="$INSTALL_PREFIX/share/gympp/gazebo/worlds:$INSTALL_PREFIX/share/gympp/gazebo/models"
        export SDF_PATH="$INSTALL_PREFIX/share/gympp/gazebo/models"
        export DYLD_LIBRARY_PATH="$TRAVIS_BUILD_DIR/build/lib:${DYLD_LIBRARY_PATH:-}"
        export PYTHONPATH=$TRAVIS_BUILD_DIR/build/bindings:${PYTHONPATH:-}
    ;;
    "windows") ;; # TODO
esac

# Install the python package
cd $TRAVIS_BUILD_DIR
pip3 install .

# Execute the pytests
cd $TRAVIS_BUILD_DIR/tests/python
pytest
