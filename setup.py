# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from setuptools import setup, find_packages

icub_model_zip = "https://github.com/diegoferigo/icub-model-pybullet/archive/master.zip"

setup(name='gym_ignition',
      version='0.1',
      author="Diego Ferigo",
      author_email="diego.ferigo@iit.it",
      description="Gym Ignition: A toolkit for developing OpenAI Gym environments "
                  "running in Ignition Gazebo.",
      url='https://github.com/robotology/gym-ignition',
      keywords="openai gym reinforcement learning environment gazebo robotics ignition",
      license="LGPL",
      platforms='any',
      python_requires='>=3.6',
      install_requires=[
            'gym >= 0.13.1',
            'numpy',
            'icub-model-pybullet @ {}'.format(icub_model_zip),
      ],
      packages=find_packages(),
      )
