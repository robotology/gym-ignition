from dataclasses import dataclass
# TODO: urdf or sdf? Urdf for iDynTree, otherwise SDF.


@dataclass
class BoxURDF:

    mass: float = 5.0
    edge: float = 0.2
    name: str = "cube_robot"

    def urdf(self) -> str:

        i = 1.0 / 12 * self.mass * (self.edge ** 2 + self.edge ** 2)
        urdf = f"""
        <robot name="{self.name}" xmlns:xacro="http://www.ros.org/wiki/xacro">
            <link name="cube">
                <inertial>
                  <origin rpy="0 0 0" xyz="0 0 0"/>
                  <mass value="{self.mass}"/>
                  <inertia ixx="{i}" ixy="0" ixz="0" iyy="{i}" iyz="0" izz="{i}"/>
                </inertial>
                <visual>
                  <geometry>
                    <box size="{self.edge} {self.edge} {self.edge}"/>
                  </geometry>
                  <origin rpy="0 0 0" xyz="0 0 0"/>
                </visual>
                <collision>
                  <geometry>
                    <box size="{self.edge} {self.edge} {self.edge}"/>
                  </geometry>
                  <origin rpy="0 0 0" xyz="0 0 0"/>
                </collision>
            </link>
        </robot>"""

        return urdf
