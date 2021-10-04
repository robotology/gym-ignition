%module(package="scenario.bindings") gazebo

%{
#define SWIG_FILE_WITH_INIT
#include "scenario/gazebo/GazeboEntity.h"
#include "scenario/gazebo/GazeboSimulator.h"
#include "scenario/gazebo/Joint.h"
#include "scenario/gazebo/Link.h"
#include "scenario/gazebo/Model.h"
#include "scenario/gazebo/utils.h"
#include "scenario/gazebo/World.h"
#include <cstdint>
%}

%naturalvar;

// Keep templated functions above the %rename directive
%inline %{
namespace scenario::gazebo::utils {
    template <typename Base, typename Derived>
    std::shared_ptr<Derived> ToGazebo(const std::shared_ptr<Base>& base)
    {
        return std::dynamic_pointer_cast<Derived>(base);
    }
}
%}

// Helpers for downcasting to Gazebo classes
%template(ToGazeboWorld) scenario::gazebo::utils::ToGazebo<scenario::core::World, scenario::gazebo::World>;
%template(ToGazeboModel) scenario::gazebo::utils::ToGazebo<scenario::core::Model, scenario::gazebo::Model>;
%template(ToGazeboJoint) scenario::gazebo::utils::ToGazebo<scenario::core::Joint, scenario::gazebo::Joint>;
%template(ToGazeboLink) scenario::gazebo::utils::ToGazebo<scenario::core::Link, scenario::gazebo::Link>;

// STL classes
%include <stdint.i>
%include <std_pair.i>
%include <std_array.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_shared_ptr.i>

// Import the module with core classes
// From http://www.swig.org/Doc4.0/Modules.html
%import "../core/core.i"

// NOTE: Keep all template instantiations above.
// Rename all methods to undercase with _ separators excluding the classes.
%rename("%(undercase)s") "";
%rename("") PID;
%rename("") Pose;
%rename("") Link;
%rename("") Joint;
%rename("") Model;
%rename("") World;
%rename("") Limit;
%rename("") Contact;
%rename("") JointType;
%rename("") Verbosity;
%rename("") JointLimit;
%rename("") ContactPoint;
%rename("") GazeboEntity;
%rename("") PhysicsEngine;
%rename("") GazeboSimulator;
%rename("") JointControlMode;

// Other templates for ScenarI/O APIs
%shared_ptr(scenario::gazebo::Joint)
%shared_ptr(scenario::gazebo::Link)
%shared_ptr(scenario::gazebo::Model)
%shared_ptr(scenario::gazebo::World)
%shared_ptr(scenario::gazebo::GazeboEntity)

// Ignored methods
%ignore scenario::gazebo::GazeboEntity::ecm;
%ignore scenario::gazebo::GazeboEntity::initialize;
%ignore scenario::gazebo::GazeboEntity::validEntity;
%ignore scenario::gazebo::GazeboEntity::eventManager;
%ignore scenario::gazebo::GazeboEntity::createECMResources;

// Workaround for https://github.com/swig/swig/issues/1830
%feature("pythonprepend") scenario::gazebo::World::getModel %{
    r"""
    Get a model part of the world.

    :type modelName: string
    :param modelName: The name of the model to get.
    :rtype: :py:class:`scenario.core.Model`
    :return: The model if it is part of the world, None otherwise.
    """
%}

// Interface of Gazebo entities
%include "scenario/gazebo/GazeboEntity.h"

// Public helpers
%include "scenario/gazebo/utils.h"

// ScenarI/O headers
%include "scenario/gazebo/Joint.h"
%include "scenario/gazebo/Link.h"
%include "scenario/gazebo/Model.h"
%include "scenario/gazebo/World.h"

// GazeboSimulator
%include "scenario/gazebo/GazeboSimulator.h"
