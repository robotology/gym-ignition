# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from dataclasses import dataclass
from gym_ignition.context import plugin


@dataclass
class DepthCamera(plugin.Plugin):  # TODO: this is not a plugin, make a sensor template?

    name: str
    width: int = 256
    height: int = 256
    fov: float = 1.05

    topic: str = ""
    # TODO: static

    def to_xml(self) -> str:  # .sdf()?
        xml = f"""
        <model name="{self.model_name}">
          <static>true</static>
    
          <link name="{self.name}_link">
            <sensor name="{self.name}" type="depth_camera">
              <always_on>1</always_on>
              <visualize>true</visualize>
              <topic>{self.topic}</topic>
              <camera>
                <horizontal_fov>{self.fov}</horizontal_fov>
                <image>
                  <width>{self.width}</width>
                  <height>{self.height}</height>
                  <format>R_FLOAT32</format>
                </image>
                <clip>
                  <near>0.1</near>
                  <far>10.0</far>
                </clip>
              </camera>
            </sensor>
            <visual name="visual">
              <geometry>
                <box>
                  <size>0.1 0.1 0.1</size>
                </box>
              </geometry>
            </visual>
          </link>

        </model>
        """

        return DepthCamera.wrap_in_sdf(xml)

    @property
    def model_name(self) -> str:
        return f"{self.name}_model"
