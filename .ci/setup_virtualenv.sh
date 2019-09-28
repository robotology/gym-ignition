#!/bin/bash
set -eu

# Read the env variable if exists, otherwise fall back to 3.6
PYTHON_EXE=python${PYTHON_VERSION:-3.6}

if [[ ! -x $(type -P ${PYTHON_EXE}) ]] ; then
    echo "Failed to find ${PYTHON_EXE} in PATH"
    exit 1
fi

# Folder of the virtualenv
VIRTUAL_ENV=/ve

# Install virtualenv
pip3 install virtualenv

# Create an empty virtualenv and enable it by default
virtualenv -p $PYTHON_EXE ${VIRTUAL_ENV}

# Enable the virtualenv without the need to activate it
export PATH=${VIRTUAL_ENV}/bin:${PATH}

# Install setuptools
pip3 install setuptools pytest
