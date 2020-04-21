# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# Workaround for https://github.com/osrf/sdformat/issues/227.
# It has to be done before loading the bindings.
import gym_ignition_models
gym_ignition_models.setup_environment()

# Import SWIG bindings
# See https://github.com/robotology/gym-ignition/issues/7
#     https://stackoverflow.com/a/45473441/12150968
import sys
if sys.platform.startswith('linux') or sys.platform.startswith('darwin'):
    import os
    dlopen_flags = sys.getdlopenflags()
    if "scenario_bindings" not in sys.modules:
        sys.setdlopenflags(dlopen_flags | os.RTLD_GLOBAL)
    else:
        sys.setdlopenflags(dlopen_flags | os.RTLD_LAZY | os.RTLD_NOLOAD | os.RTLD_GLOBAL)

    import scenario_bindings

    try:
        import gympp_bindings
    except ImportError:
        pass

    # Restore the flags
    sys.setdlopenflags(dlopen_flags)
else:
    import scenario_bindings

    try:
        import gympp_bindings
    except ImportError:
        pass

# Configure OS environment variables
from gym_ignition.utils import gazebo_env_vars, resource_finder
gazebo_env_vars.setup_gazebo_env_vars()

# Add IGN_GAZEBO_RESOURCE_PATH to the default search path
import os
if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
    resource_finder.add_path_from_env_var("IGN_GAZEBO_RESOURCE_PATH")
