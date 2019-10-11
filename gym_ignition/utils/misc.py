# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.


def installed_in_editable_mode() -> bool:
    # Note: the editable mode detection mechanism can be improved
    import gym_ignition
    import gympp_bindings
    from pathlib import Path
    return \
        Path(gympp_bindings.__file__).parent != Path(gym_ignition.__file__).parent.parent
