%pythonbegin %{
from typing import Union
%}

%extend scenario::core::World {
  %pythoncode %{
    def to_gazebo(self) -> Union["scenario.bindings.gazebo.World", "scenario.bindings.core.World"]:
        import scenario.bindings.gazebo
        return scenario.bindings.gazebo.ToGazeboWorld(self)
  %}
}

%extend scenario::core::Model {
  %pythoncode %{
    def to_gazebo(self) -> Union["scenario.bindings.gazebo.Model", "scenario.bindings.core.Model"]:
        import scenario.bindings.gazebo
        return scenario.bindings.gazebo.ToGazeboModel(self)
  %}
}

%extend scenario::core::Joint {
  %pythoncode %{
    def to_gazebo(self) -> Union["scenario.bindings.gazebo.Joint", "scenario.bindings.core.Joint"]:
        import scenario.bindings.gazebo
        return scenario.bindings.gazebo.ToGazeboJoint(self)
  %}
}

%extend scenario::core::Link {
  %pythoncode %{
    def to_gazebo(self) -> Union["scenario.bindings.gazebo.Link", "scenario.bindings.core.Link"]:
        import scenario.bindings.gazebo
        return scenario.bindings.gazebo.ToGazeboLink(self)
  %}
}
