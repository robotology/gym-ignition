# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import platform
import subprocess
from setuptools import setup, find_packages, Extension
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    """
    Custom setuptool extension to define parameters for configuring CMake.
    """

    def __init__(self,
                 name: str,
                 cmake_configuration: str,
                 source_dir: str = ''):
        Extension.__init__(self, name, sources=[])
        self.source_dir = os.path.abspath(source_dir)
        self.cmake_configuration = cmake_configuration


class BuildExtension(build_ext):
    """
    Setuptool build extension handler.
    It processes all the extensions listed in the 'ext_modules' entry.
    """

    # TODO: the help description will contain both the old -D and the new -D
    user_options = build_ext.user_options + \
                   [("define=", "D", "Create or update CMake cache")]

    def initialize_options(self):

        # Initialize base class
        build_ext.initialize_options(self)

        # Override define. This is supposed to pass C preprocessor macros, but we use it
        # to pass custom options to CMake.
        self.define = None

    def finalize_options(self):

        # Parse the custom CMake options and store them in a new attribute
        self.cmake_defines = self.define.split(";") if self.define is not None else []
        self.cmake_defines = [f"-D{define}" for define in self.cmake_defines]

        # Call base class
        build_ext.finalize_options(self)

    def run(self):
        try:
            _ = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: " +
                ", ".join(e.name for e in self.extensions))

        if len(self.extensions) != 1 or not \
                isinstance(self.extensions[0], CMakeExtension):
            raise RuntimeError("This class can only build one CMakeExtension object")

        if platform.system() != "Linux":
            raise RuntimeError("Only Linux is currently supported")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        # In editable mode, do not compile C++.
        # Developers should install the project through regular CMake.
        # Note that here we use the 'inplace' variable that it switched on by the
        # pip '--editable' option.
        if self.inplace:
            print("Editable install recognized. The CMake project will not be installed.")
            return

        # Get the temporary external build directory.
        # On Ubuntu it will be '/tmp/pip-req-build-<randomseq>/build/linux-x86_64-3.6'.
        ext_dir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # Shared CMake arguments
        cmake_args = [
            f"-DCMAKE_INSTALL_PREFIX:PATH={ext_dir}",
        ]

        # Apply the selected CMake build type
        cfg = ext.cmake_configuration
        build_args = ['--config', cfg]

        # Handle differences in OS and CMake generators
        if platform.system() == "Windows":
            # Refer to https://github.com/pybind/cmake_example/blob/master/setup.py
            raise NotImplementedError
        elif platform.system() == "Darwin":
            raise NotImplementedError
        elif platform.system() == "Linux":
            cmake_args += [
                f"-GNinja",
                f"-DCMAKE_BUILD_TYPE={cfg}",
            ]
            install_target = "install"
        else:
            raise RuntimeError(f"Unsupported '{platform.system()}' platform")

        # Parse the optional CMake options. They can be passed as:
        #
        # python setup.py build_ext -D"BAR=Foo;VAR=TRUE"
        # python setup.py bdist_wheel build_ext -D"BAR=Foo;VAR=TRUE"
        # python setup.py install build_ext -D"BAR=Foo;VAR=TRUE"
        # python setup.py install -e build_ext -D"BAR=Foo;VAR=TRUE"
        # pip install --global-option="build_ext" --global-option="-DBAR=Foo;VAR=TRUE" .
        #
        cmake_args += self.cmake_defines

        # Commands to be executed
        configure_command = ['cmake', ext.source_dir] + cmake_args
        build_command = ['cmake', '--build', '.'] + build_args
        install_command = ['cmake', '--build', '.', '--target', install_target]

        print("")
        print(f"Build folder: {self.build_temp}")
        print("")
        print(f"1) Configuring: {configure_command}")
        print(f"2) Building   : {build_command}")
        print(f"3) Installing : {install_command}")
        print("")

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        # Call CMake
        subprocess.check_call(configure_command, cwd=self.build_temp)
        subprocess.check_call(build_command, cwd=self.build_temp)
        subprocess.check_call(install_command, cwd=self.build_temp)


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
        "Development Status :: 4 - Beta",
        "Operating System :: POSIX :: Linux",
        "Topic :: Games/Entertainment :: Simulation",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Framework :: Robot Framework",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Programming Language :: C++",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3 :: Only",
        "License :: OSI Approved :: GNU General Public License v2 or later (GPLv2+)",
    ],
    use_scm_version={
        'local_scheme': 'dirty-tag',
    },
    setup_requires=['setuptools_scm'],
    python_requires='>=3.6',
    install_requires=[
        'gym >= 0.13.1',
        'numpy',
        'gym_ignition_models',
        'lxml',
    ],
    extras_require=dict(
        website=[
            "sphinx",
            "sphinx-rtd-theme",
            "sphinx-autodoc-typehints",
            "sphinx_fontawesome",
            "sphinx-multiversion",
            "breathe",
        ],
    ),
    packages=find_packages("python"),
    package_dir={'': "python"},
    ext_modules=[CMakeExtension(name='InstallAllTargets', cmake_configuration='PyPI')],
    cmdclass={
        'build_ext': BuildExtension,
    },
    zip_safe=False,
)
