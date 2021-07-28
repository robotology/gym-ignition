# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import pytest

pytestmark = pytest.mark.gym_ignition

import gym_ignition_models
from gym_ignition.randomizers.model import sdf
from gym_ignition.randomizers.model.sdf import (
    Distribution,
    GaussianParams,
    Method,
    UniformParams,
)
from gym_ignition.utils import misc
from lxml import etree

from scenario import gazebo as scenario


def test_sdf_randomizer():

    # Get the URDF model
    urdf_model = gym_ignition_models.get_model_file("cartpole")

    # Convert it to a SDF string
    sdf_model_string = scenario.urdffile_to_sdfstring(urdf_model)

    # Write the SDF string to a temp file
    sdf_model = misc.string_to_file(sdf_model_string)

    # Create the randomizer
    randomizer = sdf.SDFRandomizer(sdf_model=sdf_model)

    # Get the original model string. It is parsed and then serialized without changes.
    orig_model = randomizer.sample(pretty_print=True)

    with pytest.raises(ValueError):
        # Setting wrong distribution
        randomizer.new_randomization().at_xpath(
            "*/link[@name='pole']/inertial/inertia/ixx"
        ).method(Method.Additive).sampled_from(
            Distribution.Uniform, GaussianParams(mean=0, variance=0.1)
        ).add()

    # Add a uniform randomization
    randomizer.new_randomization().at_xpath(
        "*/link[@name='pole']/inertial/inertia/ixx"
    ).method(Method.Additive).sampled_from(
        Distribution.Gaussian, GaussianParams(mean=0, variance=0.1)
    ).add()

    randomizer.process_data()
    assert len(randomizer.get_active_randomizations()) == 1

    assert randomizer.sample(pretty_print=True) != orig_model

    # Clean the randomizer
    randomizer.clean()
    assert len(randomizer.get_active_randomizations()) == 0
    assert randomizer.sample(pretty_print=True) == orig_model

    # Add a multi-match randomization
    randomizer.new_randomization().at_xpath("*/link/inertial/inertia/ixx").method(
        Method.Coefficient
    ).sampled_from(Distribution.Uniform, UniformParams(low=0.8, high=1.2)).add()

    assert len(randomizer.get_active_randomizations()) == 1

    # Expand the matches
    randomizer.process_data()
    assert len(randomizer.get_active_randomizations()) > 1

    # Sample
    assert randomizer.sample(pretty_print=True) != orig_model


def test_randomizer_reproducibility():

    # Get the model
    sdf_model = gym_ignition_models.get_model_file("ground_plane")

    # Initialize the randomizers
    randomizer1 = sdf.SDFRandomizer(sdf_model=sdf_model)
    randomizer2 = sdf.SDFRandomizer(sdf_model=sdf_model)
    randomizer3 = sdf.SDFRandomizer(sdf_model=sdf_model)

    # Randomize the ground friction of all links (the ground plane collision)
    frictions = randomizer1.find_xpath("*/link/collision/surface/friction")
    assert len(frictions) == 1

    # Get the original model string. It is parsed and then serialized without changes.
    orig_model_string1 = randomizer1.sample(pretty_print=True)
    orig_model_string2 = randomizer2.sample(pretty_print=True)
    orig_model_string3 = randomizer3.sample(pretty_print=True)
    assert orig_model_string1 == orig_model_string2 == orig_model_string3

    # Do not seed #3
    randomizer1.seed(42)
    randomizer2.seed(42)

    # Add randomizations for #1
    randomizer1.new_randomization().at_xpath(
        "*/link/collision/surface/friction/ode/mu"
    ).method(Method.Absolute).sampled_from(
        Distribution.Uniform, UniformParams(low=0, high=100)
    ).add()

    # Add randomizations for #2
    randomizer2.new_randomization().at_xpath(
        "*/link/collision/surface/friction/ode/mu"
    ).method(Method.Absolute).sampled_from(
        Distribution.Uniform, UniformParams(low=0, high=100)
    ).add()

    # Add randomizations for #3
    randomizer3.new_randomization().at_xpath(
        "*/link/collision/surface/friction/ode/mu"
    ).method(Method.Absolute).sampled_from(
        Distribution.Uniform, UniformParams(low=0, high=100)
    ).add()

    # Process the randomizations
    randomizer1.process_data()
    randomizer2.process_data()
    randomizer3.process_data()

    for _ in range(5):
        model1 = randomizer1.sample()
        model2 = randomizer2.sample()
        model3 = randomizer3.sample()

        assert model1 == model2
        assert model1 != model3


def test_randomize_missing_element():

    # Get the URDF model
    urdf_model = gym_ignition_models.get_model_file("pendulum")

    # Convert it to a SDF string
    sdf_model_string = scenario.urdffile_to_sdfstring(urdf_model)

    # Write the SDF string to a temp file
    sdf_model = misc.string_to_file(sdf_model_string)

    # Create the randomizer
    randomizer = sdf.SDFRandomizer(sdf_model=sdf_model)

    # Try to randomize a missing element
    with pytest.raises(RuntimeError):
        # The ode/mu elements are missing
        randomizer.new_randomization().at_xpath(
            "*/link/collision/surface/friction/ode/mu"
        ).method(Method.Absolute).sampled_from(
            Distribution.Uniform, UniformParams(low=0, high=100)
        ).add()

    # Add the missing friction/ode/mu element. We assume that friction exists.
    frictions = randomizer.find_xpath("*/link/collision/surface/friction")

    for friction in frictions:

        # Create parent 'ode' first
        if friction.find("ode") is None:
            etree.SubElement(friction, "ode")

        # Create child 'mu' after
        ode = friction.find("ode")
        if ode.find("mu") is None:
            etree.SubElement(ode, "mu")

        # Assign a dummy value to mu
        mu = ode.find("mu")
        mu.text = str(0)

    # Apply the same randomization
    randomizer.new_randomization().at_xpath(
        "*/link/collision/surface/friction/ode/mu"
    ).method(Method.Absolute).sampled_from(
        Distribution.Uniform, UniformParams(low=0, high=100)
    ).ignore_zeros(
        False
    ).add()

    # Process the randomization and sample a model
    randomizer.process_data()

    model1 = randomizer.sample(pretty_print=True)
    model2 = randomizer.sample(pretty_print=True)
    assert model1 != model2


def test_full_panda_randomization():

    # Get the URDF model
    urdf_model = gym_ignition_models.get_model_file("panda")

    # Convert it to a SDF string
    sdf_model_string = scenario.urdffile_to_sdfstring(urdf_model)

    # Write the SDF string to a temp file
    sdf_model = misc.string_to_file(sdf_model_string)

    # Create the randomizer
    randomizer = sdf.SDFRandomizer(sdf_model=sdf_model)

    joint_dynamics = randomizer.find_xpath("*/joint/axis/dynamics")
    assert len(joint_dynamics) > 0

    # Add the friction and damping elements since they're missing in the model
    for joint_dynamic in joint_dynamics:

        if joint_dynamic.find("friction") is None:
            etree.SubElement(joint_dynamic, "friction")
            friction = joint_dynamic.find("friction")
            friction.text = str(0)

        if joint_dynamic.find("damping") is None:
            etree.SubElement(joint_dynamic, "damping")
            damping = joint_dynamic.find("damping")
            damping.text = str(3)

    randomization_config = {
        "*/link/inertial/mass": {
            # mass + U(-0.5, 0.5)
            "method": Method.Additive,
            "distribution": Distribution.Uniform,
            "params": UniformParams(low=-0.5, high=0.5),
            "ignore_zeros": True,
            "force_positive": True,
        },
        "*/link/inertial/inertia/ixx": {
            # inertia * N(1, 0.2)
            "method": Method.Coefficient,
            "distribution": Distribution.Gaussian,
            "params": GaussianParams(mean=1.0, variance=0.2),
            "ignore_zeros": True,
            "force_positive": True,
        },
        "*/link/inertial/inertia/iyy": {
            "method": Method.Coefficient,
            "distribution": Distribution.Gaussian,
            "params": GaussianParams(mean=1.0, variance=0.2),
            "ignore_zeros": True,
            "force_positive": True,
        },
        "*/link/inertial/inertia/izz": {
            "method": Method.Coefficient,
            "distribution": Distribution.Gaussian,
            "params": GaussianParams(mean=1.0, variance=0.2),
            "ignore_zeros": True,
            "force_positive": True,
        },
        "*/joint/axis/dynamics/friction": {
            # friction in [0, 5]
            "method": Method.Absolute,
            "distribution": Distribution.Uniform,
            "params": UniformParams(low=0, high=5),
            "ignore_zeros": False,  # We initialized the value as 0
            "force_positive": True,
        },
        "*/joint/axis/dynamics/damping": {
            # damping (= 3.0) * [0.8, 1.2]
            "method": Method.Coefficient,
            "distribution": Distribution.Uniform,
            "params": UniformParams(low=0.8, high=1.2),
            "ignore_zeros": True,
            "force_positive": True,
        },
        # TODO: */joint/axis/limit/effort
    }

    for xpath, config in randomization_config.items():

        randomizer.new_randomization().at_xpath(xpath).method(
            config["method"]
        ).sampled_from(config["distribution"], config["params"]).force_positive(
            config["distribution"]
        ).ignore_zeros(
            config["ignore_zeros"]
        ).add()

    randomizer.process_data()
    assert len(randomizer.get_active_randomizations()) > 0

    randomizer.sample(pretty_print=True)
