# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import abc
from typing import Tuple
from dataclasses import dataclass, field

# Default SDF version used in the serialized XML context
SDF_VERSION=1.7

# Read the following for more information about dataclasses internals:
# https://florimond.dev/blog/articles/2018/10/reconciling-dataclasses-and-properties-in-python/


@dataclass
class GazeboPlugin(abc.ABC):
    """
    Base class of all World and Model plugins for Ignition Gazebo.

    The Plugin abstract class provides boilerplate code that simplifies and streamlines
    the definition of helper classes that insert Ignition Gazebo plugins to either World
    or Model objects.

    Classes that inherit from Plugin have to provide the following information:

    1) All the properties of the plugin as dataclass fields
    2) The private specification of the plugin (plugin name and plugin class)
    3) Optionally: the serialized XML context

    Example:

    .. code-block:: python

        plugin = MyPlugin(my_property=42)
        model = MyModel(world=my_world)

        model.insert_model_plugin(*plugin.args())
    """

    _plugin_name: str = field(init=False, repr=False)
    _plugin_class: str = field(init=False, repr=False)

    def to_xml(self) -> str:
        """
        Get the XML plugin content.

        Returns:
            The XML plugin content.
        """
        return ""

    def args(self) -> Tuple[str, str, str]:
        """
        Get the arguments passed to the ScenarI/O methods used to insert plugins.

        Returns:
            A tuple with the args required to insert the plugin.
        """
        return str(self._plugin_name), \
               str(self._plugin_class), \
               GazeboPlugin.wrap_in_sdf(self.to_xml())

    @staticmethod
    def wrap_in_sdf(context: str) -> str:
        """
        Wrap the XML context inside a SDF root.

        Args:
            context: The XML string with the plugin's context.

        Returns:
            The plugin's context wrapped inside a SDF root.
        """

        return f"""<sdf version='{SDF_VERSION}'>{context}</sdf>"""
