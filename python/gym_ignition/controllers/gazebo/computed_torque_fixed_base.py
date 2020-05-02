# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from typing import List, NamedTuple


class ComputedTorqueFixedBaseContext(NamedTuple):
    name: str
    kp: List[float]
    ki: List[float]
    kd: List[float]
    urdf: str
    joints: List[str]
    gravity: List[float]

    def to_xml(self) -> str:
        xml = f"""
        <sdf version='1.7'>
            <name>foo</name>
            <test arg='boh'/>
            <controller name="{self.name}">
                <kp>{self._to_str(self.kp)}</kp>
                <ki>{self._to_str(self.ki)}</ki>
                <kd>{self._to_str(self.kd)}</kd>
                <urdf>{self.urdf}</urdf>
                <joints>{self._to_str(self.joints)}</joints>
                <gravity>{self._to_str(self.gravity)}</gravity>
            </controller>
            <!--test/-->
        </sdf>
        """

        return xml

    @staticmethod
    def _to_str(input_list: List):

        list_serialized = ""

        for index, element in enumerate(input_list[:-1]):
            list_serialized += str(element) + " "

        list_serialized += str(input_list[-1])

        return list_serialized