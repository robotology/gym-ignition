%module(moduleimport="import $module") scenario_bindings

%{
#define SWIG_FILE_WITH_INIT
#include "scenario/gazebo/GazeboSimulator.h"
#include "scenario/gazebo/Joint.h"
#include "scenario/gazebo/Link.h"
#include "scenario/gazebo/Model.h"
#include "scenario/gazebo/utils.h"
#include "scenario/gazebo/World.h"
#include "scenario/plugins/gazebo/ECMSingleton.h"
#include <cstdint>
%}

%naturalvar;

// STL classes
%include <stdint.i>
%include <std_pair.i>
%include <std_array.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_shared_ptr.i>

// Convert python list to std::vector
%template(Vector_i) std::vector<int>;
%template(Vector_u) std::vector<size_t>;
%template(Vector_f) std::vector<float>;
%template(Vector_d) std::vector<double>;
%template(Vector_s) std::vector<std::string>;

// Convert python list to std::array
%template(Array3d) std::array<double, 3>;
%template(Array4d) std::array<double, 4>;
%template(Array6d) std::array<double, 6>;

// Pair instantiation
%template(PosePair) std::pair<std::array<double, 3>, std::array<double, 4>>;

// Public helpers
%include "scenario/gazebo/utils.h"

// Other templates for ScenarI/O APIs
%shared_ptr(scenario::gazebo::Joint)
%shared_ptr(scenario::gazebo::Link)
%shared_ptr(scenario::gazebo::Model)
%shared_ptr(scenario::gazebo::World)
%template(Vector_contact) std::vector<scenario::base::Contact>;
%template(Vector_contact_point) std::vector<scenario::base::ContactPoint>;

// Ignored methods
%ignore scenario::gazebo::Joint::initialize;
%ignore scenario::gazebo::Link::initialize;
%ignore scenario::gazebo::Model::initialize;
%ignore scenario::gazebo::World::initialize;
%ignore scenario::gazebo::Joint::createECMResources;
%ignore scenario::gazebo::Link::createECMResources;
%ignore scenario::gazebo::Model::createECMResources;
%ignore scenario::gazebo::World::createECMResources;

// ScenarI/O headers
%include "scenario/gazebo/Joint.h"
%include "scenario/gazebo/Link.h"
%include "scenario/gazebo/Model.h"
%include "scenario/gazebo/World.h"

// GazeboSimulator
%shared_ptr(scenario::gazebo::GazeboSimulator)
%include "scenario/gazebo/GazeboSimulator.h"

// ECMSingleton
%ignore scenario::plugins::gazebo::ECMSingleton::clean;
%ignore scenario::plugins::gazebo::ECMSingleton::getECM;
%ignore scenario::plugins::gazebo::ECMSingleton::getEventManager;
%ignore scenario::plugins::gazebo::ECMSingleton::storePtrs;
%include "scenario/plugins/gazebo/ECMSingleton.h"
