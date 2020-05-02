%module gympp_bindings

%{
#define SWIG_FILE_WITH_INIT
#include "gympp/base/Common.h"
#include "gympp/base/Environment.h"
#include "gympp/base/Space.h"
#include "gympp/gazebo/GazeboEnvironment.h"
#include "gympp/gazebo/GymFactory.h"
#include "gympp/gazebo/Metadata.h"
#include "scenario/gazebo/GazeboSimulator.h"
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
%template(VectorI) std::vector<int>;
%template(VectorU) std::vector<size_t>;
%template(VectorF) std::vector<float>;
%template(VectorD) std::vector<double>;
%template(VectorS) std::vector<std::string>;

// Convert python list to std::array
%template(Array3d) std::array<double, 3>;
%template(Array4d) std::array<double, 4>;
%template(Array6d) std::array<double, 6>;

%shared_ptr(gympp::base::spaces::Space)
%shared_ptr(gympp::base::spaces::Box)
%shared_ptr(gympp::base::spaces::Discrete)

%include "gympp/base/Common.h"
%include "gympp/base/Space.h"
%template(get_i) gympp::base::data::Sample::get<int>;
%template(get_u) gympp::base::data::Sample::get<size_t>;
%template(get_f) gympp::base::data::Sample::get<float>;
%template(get_d) gympp::base::data::Sample::get<double>;
%template(BufferContainerI) gympp::base::BufferContainer<int>;
%template(BufferContainerU) gympp::base::BufferContainer<size_t>;
%template(BufferContainerF) gympp::base::BufferContainer<float>;
%template(BufferrContainerD) gympp::base::BufferContainer<double>;
%template(get_buffer_i) gympp::base::data::Sample::getBuffer<int>;
%template(get_buffer_u) gympp::base::data::Sample::getBuffer<size_t>;
%template(get_buffer_f) gympp::base::data::Sample::getBuffer<float>;
%template(get_buffer_d) gympp::base::data::Sample::getBuffer<double>;

%include "optional.i"
%template(OptionalI) std::optional<int>;
%template(OptionalU) std::optional<size_t>;
%template(OptionalF) std::optional<float>;
%template(OptionalD) std::optional<double>;
%template(OptionalState) std::optional<gympp::base::State>;
%template(OptionalSample) std::optional<gympp::base::data::Sample>;

%include "ignition/common/SingletonT.hh"
%ignore ignition::common::SingletonT<gympp::gazebo::GymFactory>::myself;
%template(GymFactorySingleton) ignition::common::SingletonT<gympp::gazebo::GymFactory>;

%rename("%(undercase)s") "";
%rename("") Space;
%rename("") State;
%rename("") Range;
%rename("") Limit;
%rename("") Sample;
%rename("") Buffer;
%rename("") SpaceType;
%rename("") RenderMode;
%rename("") GymFactory;
%rename("") PhysicsData;
%rename("") Environment;
%rename("") ActionSpace;
%rename("") RewardRange;
%rename("") ModelInitData;
%rename("") SpaceMetadata;
%rename("") PluginMetadata;
%rename("") BufferContainer;
%rename("") GazeboSimulator;
%rename("") ObservationSpace;
%rename("") GazeboEnvironment;
%rename("") gympp::base::spaces::Box;
%rename("") gympp::base::spaces::Discrete;

%shared_ptr(scenario::gazebo::GazeboSimulator)
%include "scenario/gazebo/GazeboSimulator.h"

%shared_ptr(gympp::base::Environment)
%shared_ptr(gympp::gazebo::GazeboEnvironment)

%include "gympp/base/Environment.h"
%include "gympp/gazebo/GazeboEnvironment.h"

%inline %{
    std::shared_ptr<gympp::gazebo::GazeboEnvironment> envToGazeboEnvironment(gympp::base::EnvironmentPtr env) {
        return std::dynamic_pointer_cast<gympp::gazebo::GazeboEnvironment>(env);
    }

    std::shared_ptr<scenario::gazebo::GazeboSimulator> envToGazeboSimulator(gympp::base::EnvironmentPtr env) {
        return std::dynamic_pointer_cast<scenario::gazebo::GazeboSimulator>(env);
    }
%}

%include "gympp/gazebo/Metadata.h"
%include "gympp/gazebo/GymFactory.h"
