# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from pathlib import Path
from setuptools import setup, find_packages

# Read the contents of the README file
this_directory = Path(__file__).absolute().parent
with open(this_directory / "README.md", encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="gym-ignition",
    author="Diego Ferigo",
    author_email="diego.ferigo@iit.it",
    description="Gym-Ignition: A toolkit for developing OpenAI Gym environments "
                "simulated with Ignition Gazebo.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/robotology/gym-ignition",
    project_urls={
        "Bug Tracker": "https://github.com/robotology/gym-ignition/issues",
        "Documentation": "https://robotology.github.io/gym-ignition",
        "Source Code": "https://github.com/robotology/gym-ignition",
    },
    keywords=["openai", "gym", "reinforcement learning", "rl", "environment", "gazebo",
              "robotics", "ignition", "humanoid", "panda", "icub", "urdf", "sdf"],
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
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3 :: Only",
        "License :: OSI Approved :: GNU Lesser General Public License v2 or later (LGPLv2+)",
    ],
    use_scm_version=dict(local_scheme="dirty-tag"),
    setup_requires=[
        "wheel",
        "setuptools_scm",
        "cmake-build-extension",
    ],
    python_requires=">=3.8",
    install_requires=[
        "scenar-io",  # temporary name waiting for 'scenario'
        "gym>=0.13.1",
        "numpy",
        "gym_ignition_models",
        "lxml",
        "idyntree",
    ],
    extras_require=dict(
        website=[
            "sphinx",
            "sphinx-book-theme",
            "sphinx-autodoc-typehints",
            "sphinx_fontawesome",
            "sphinx-multiversion",
            "sphinx-tabs",
            "breathe",
        ],
        test=[
            "pytest",
            "pytest-xvfb",
            "pytest-icdiff",
        ]
    ),
    packages=find_packages("python"),
    package_dir={"": "python"},
    zip_safe=False,
)
