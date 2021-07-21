%module(package="scenario.bindings") core

%{
#define SWIG_FILE_WITH_INIT
#include "scenario/core/Joint.h"
#include "scenario/core/Link.h"
#include "scenario/core/Model.h"
#include "scenario/core/World.h"
#include "scenario/core/utils/utils.h"
%}

%naturalvar;

// Convert all exceptions to RuntimeError
%include "exception.i"
%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}

// STL classes
%include <stdint.i>
%include <std_pair.i>
%include <std_array.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_shared_ptr.i>

// Convert python list to std::vector
%template(VectorI) std::vector<int>;
%template(VectorU) std::vector<size_t>;
%template(VectorF) std::vector<float>;
%template(VectorD) std::vector<double>;
%template(VectorS) std::vector<std::string>;

// Convert python list to std::array
%template(Array3d) std::array<double, 3>;
%template(Array4d) std::array<double, 4>;
%template(Array6d) std::array<double, 6>;

// Pair instantiation
%template(PosePair) std::pair<std::array<double, 3>, std::array<double, 4>>;

// ScenarI/O templates
%template(VectorOfLinks) std::vector<scenario::core::LinkPtr>;
%template(VectorOfJoints) std::vector<scenario::core::JointPtr>;
%template(VectorOfContacts) std::vector<scenario::core::Contact>;
%template(VectorOfContactPoints) std::vector<scenario::core::ContactPoint>;

// Doxygen typemaps
%typemap(doctype) std::array<double, 3> "Tuple[float, float, float]";
%typemap(doctype) std::array<double, 4> "Tuple[float, float, float, float]";
%typemap(doctype) std::array<double, 6> "Tuple[float, float, float, float, float, float]";
%typemap(doctype) std::vector<double> "Tuple[float]";
%typemap(doctype) std::vector<std::string> "Tuple[string]";
%typemap(doctype) std::vector<scenario::core::LinkPtr> "Tuple[Link]";
%typemap(doctype) std::vector<scenario::core::JointPtr> "Tuple[Joint]";
%typemap(doctype) std::vector<scenario::core::Contact> "Tuple[Contact]";
%typemap(doctype) std::vector<scenario::core::ContactPoint> "Tuple[ContactPoint]";

%pythonbegin %{
from typing import Tuple
%}

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
%rename("") JointLimit;
%rename("") ContactPoint;
%rename("") JointControlMode;

// Public helpers
%include "scenario/core/utils/utils.h"

// Other templates for ScenarI/O APIs
%shared_ptr(scenario::core::Joint)
%shared_ptr(scenario::core::Link)
%shared_ptr(scenario::core::Model)
%shared_ptr(scenario::core::World)

// ScenarI/O core headers
%include "scenario/core/Joint.h"
%include "scenario/core/Link.h"
%include "scenario/core/Model.h"
%include "scenario/core/World.h"

// Downcast pointers to the implementation classes
#if defined (SCENARIO_HAS_GAZEBO)
%include "../gazebo/to_gazebo.i"
#endif
