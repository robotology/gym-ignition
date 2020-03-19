%module scenario_bindings

%{
#define SWIG_FILE_WITH_INIT
#include "scenario/gazebo/GazeboSimulator.h"
#include "scenario/gazebo/Joint.h"
#include "scenario/gazebo/Link.h"
#include "scenario/gazebo/Model.h"
#include "scenario/gazebo/utils.h"
#include "scenario/gazebo/World.h"
#include <cstdint>
%}

%naturalvar;

// STL classes
%include <stdint.i>
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

%include "scenario/gazebo/utils.h"

%shared_ptr(scenario::gazebo::GazeboSimulator)
%include "scenario/gazebo/GazeboSimulator.h"

%shared_ptr(scenario::gazebo::Joint)
%shared_ptr(scenario::gazebo::Link)
%shared_ptr(scenario::gazebo::Model)
%shared_ptr(scenario::gazebo::World)

%template(Vector_contact) std::vector<scenario::base::ContactData>;
%include "scenario/gazebo/Joint.h"
%include "scenario/gazebo/Link.h"
%include "scenario/gazebo/Model.h"
%include "scenario/gazebo/World.h"
