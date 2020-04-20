# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import tempfile


def string_to_file(string: str) -> str:

    handle, tmpfile = tempfile.mkstemp()

    with open(handle, 'w') as f:
        f.write(string)

    return tmpfile
