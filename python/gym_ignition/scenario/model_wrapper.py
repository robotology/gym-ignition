# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
from gym_ignition import scenario_bindings as bindings


class ModelWrapper(bindings.Model, abc.ABC):

    def __init__(self, model: bindings.Model):

        # No need to call bindings.Model.__init__()!
        abc.ABC.__init__(self)

        self.model = model

    def __getattr__(self, name):

        return getattr(self.model, name)
