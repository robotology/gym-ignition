#!/bin/bash
set -e

if [ ! -x "/setup_virtualenv.sh" ] ; then
    echo "File '/setup_virtualenv.sh' not found."
    exit 1
fi

# Setup the python virtualenv
bash /setup_virtualenv.sh

# If a CMD is passed, execute it
exec "$@"
