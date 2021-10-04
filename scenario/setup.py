# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import setuptools
from cmake_build_extension import BuildExtension, CMakeExtension

setuptools.setup(
    ext_modules=[
        CMakeExtension(
            name="ScenarioCMakeProject",
            install_prefix="scenario",
            cmake_build_type="PyPI",
            cmake_depends_on=["idyntree"],
            disable_editable=True,
        )
    ],
    cmdclass=dict(build_ext=BuildExtension),
)
