# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from pathlib import Path
from setuptools import setup
from cmake_build_extension import BuildExtension, CMakeExtension

# Read the contents of the README file
this_directory = Path(__file__).absolute().parent
with open(this_directory / "README.md", encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="scenar-io",  # temporary name waiting for 'scenario'
    author="Diego Ferigo",
    author_email="diego.ferigo@iit.it",
    description="SCENe interfAces for Robot Input/Output.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/robotology/gym-ignition/tree/master/scenario",
    project_urls={
        "Bug Tracker": "https://github.com/robotology/gym-ignition/issues",
        "Documentation": "https://robotology.github.io/gym-ignition",
        "Source Code": "https://github.com/robotology/gym-ignition/tree/master/scenario",
    },
    keywords=["robotics", "gazebo", "ignition", "simulation", "physics", "multibody",
              "dynamics", "physics simulation", "middleware", "real-time"],
    license="LGPL",
    platforms="any",
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Operating System :: POSIX :: Linux",
        "Topic :: Games/Entertainment :: Simulation",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Scientific/Engineering :: Physics",
        "Topic :: Software Development",
        "Framework :: Robot Framework",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Programming Language :: C++",
        "Programming Language :: Python :: 3 :: Only",
        "License :: OSI Approved :: GNU Lesser General Public License v2 or later (LGPLv2+)",
    ],
    use_scm_version=dict(local_scheme="dirty-tag", root="..", relative_to=__file__),
    setup_requires=[
        "wheel",
        "setuptools_scm",
        "cmake-build-extension",
        "idyntree>=3.1",
    ],
    python_requires=">=3.0",
    ext_modules=[CMakeExtension(name="ScenarioCMakeProject",
                                install_prefix="scenario",
                                cmake_build_type="PyPI",
                                cmake_depends_on=["idyntree"],
                                disable_editable=True)],
    cmdclass=dict(build_ext=BuildExtension),
    zip_safe=False,
)
