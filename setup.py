# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
from setuptools import setup, find_packages
from cmake_build_extension import BuildExtension, CMakeExtension

# Read the contents of your README file
this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='gym-ignition',
    author="Diego Ferigo",
    author_email="diego.ferigo@iit.it",
    description="Gym-Ignition: A toolkit for developing OpenAI Gym environments "
                "simulated with Ignition Gazebo.",
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/robotology/gym-ignition',
    keywords="openai gym reinforcement learning environment gazebo robotics ignition",
    license="LGPL",
    platforms='any',
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Operating System :: POSIX :: Linux",
        "Topic :: Games/Entertainment :: Simulation",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Framework :: Robot Framework",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Programming Language :: C++",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3 :: Only",
        "License :: OSI Approved :: GNU Lesser General Public License v2 or later (LGPLv2+)",
    ],
    use_scm_version={
        'local_scheme': 'dirty-tag',
    },
    setup_requires=['setuptools_scm'],
    python_requires='>=3.8',
    install_requires=[
        'gym >= 0.13.1',
        'numpy',
        'gym_ignition_models',
        'lxml',
        'idyntree',
    ],
    extras_require=dict(
        website=[
            "sphinx",
            "sphinx-book-theme < 0.0.39",
            "sphinx-autodoc-typehints",
            "sphinx_fontawesome",
            "sphinx-multiversion",
            'sphinx-tabs',
            "breathe",
        ],
        test=[
            "pytest",
            "pytest-xvfb",
            "pytest-icdiff",
        ]
    ),
    packages=find_packages("python"),
    package_dir={'': "python"},
    ext_modules=[CMakeExtension(name='InstallAllTargets',
                                install_prefix="scenario",
                                cmake_build_type='PyPI',
                                cmake_depends_on=["idyntree"],
                                disable_editable=True)],
    cmdclass={
        'build_ext': BuildExtension,
    },
    zip_safe=False,
)
