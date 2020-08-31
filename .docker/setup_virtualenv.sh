#!/bin/bash
set -eu

# Read the env variable if exists, otherwise fall back to python3
PYTHON_EXE=python${PYTHON_VERSION:-3}

if [ ! -x $(type -P ${PYTHON_EXE}) ] ; then
    echo "Failed to find ${PYTHON_EXE} in PATH"
    exit 1
fi

# Create an empty virtualenv and enable it by default
virtualenv -p $PYTHON_EXE ${VIRTUAL_ENV}

# Install pytest in the virtual environment
${VIRTUAL_ENV}/bin/pip3 install pytest
