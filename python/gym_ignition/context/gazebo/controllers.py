# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from gym_ignition.context import gazebo
from typing import Iterable, List, Tuple
from dataclasses import dataclass, field

GRAVITY = (0, 0, -9.80665)


@dataclass
class ComputedTorqueFixedBase(gazebo.plugin.GazeboPlugin):

    urdf: str
    kp: List[float]
    ki: List[float]
    kd: List[float]
    joints: List[str]
    gravity: Tuple[float, float, float] = field(default_factory=lambda: GRAVITY)

    # Private fields
    _name: str = field(init=False, repr=False, default="ComputedTorqueFixedBase")
    _plugin_name: str = field(init=False, repr=False, default="ControllerRunner")
    _plugin_class: str = field(init=False,
                               repr=False,
                               default="scenario::plugins::gazebo::ControllerRunner")

    def to_xml(self) -> str:
        xml = f"""
        <controller name="{self._name}">
            <kp>{self._to_str(self.kp)}</kp>
            <ki>{self._to_str(self.ki)}</ki>
            <kd>{self._to_str(self.kd)}</kd>
            <urdf>{self.urdf}</urdf>
            <joints>{self._to_str(self.joints)}</joints>
            <gravity>{self._to_str(self.gravity)}</gravity>
        </controller>
        """

        return xml

    @staticmethod
    def _to_str(iterable: Iterable) -> str:

        return " ".join([str(el) for el in iterable])
