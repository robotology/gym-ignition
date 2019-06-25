/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/gazebo/GazeboWrapper.h"
#include "gympp/Log.h"
#include "process.hpp"

#include <ignition/common/Console.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/config.hh>
#include <sdf/Element.hh>
#include <sdf/Error.hh>
#include <sdf/Model.hh>
#include <sdf/Physics.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <cassert>
#include <chrono>

using namespace gympp::gazebo;

// =====
// PIMPL
// =====

struct GazeboData
{
    PhysicsData physics;
    uint64_t numOfIterations = 0;
    ignition::gazebo::ServerConfig config;
    std::unique_ptr<TinyProcessLib::Process> gui;
    std::shared_ptr<ignition::gazebo::Server> server;
};

class GazeboWrapper::Impl
{
public:
    size_t id;

    GazeboData gazebo;
    std::shared_ptr<ignition::gazebo::Server> getServer();

    sdf::Root sdf;
    bool findAndLoadSdf(const std::string& sdfFileName, sdf::Root& sdfRoot);

    PhysicsData getPhysicsData() const;
    bool setPhysics(const PhysicsData& physicsData);

    ignition::common::SystemPaths systemPaths;

    std::string scopedModelName;
};

std::shared_ptr<ignition::gazebo::Server> GazeboWrapper::Impl::getServer()
{
    // Lazy initialization of the server
    if (!gazebo.server) {

        assert(sdf.Element());
        assert(!sdf.Element()->ToString("").empty());

        // Check if there are sdf parsing errors
        sdf::Root root;
        auto errors = root.LoadSdfString(sdf.Element()->ToString(""));
        assert(errors.empty()); // This should be already ok

        // Store the sdf string in the gazebo configuration
        gazebo.config.SetSdfString(sdf.Element()->ToString(""));

        // Create the server
        gazebo.config.SetUseLevels(false);
        gazebo.server = std::make_shared<ignition::gazebo::Server>(gazebo.config);
        assert(gazebo.server);

        // The GUI needs the server already up. Warming up the first iteration and pausing the
        // server. It will be unpaused at the step() call.
        gymppDebug << "Starting the server as paused" << std::endl;
        if (!gazebo.server->Run(/*blocking=*/false, gazebo.numOfIterations, /*paused=*/true)) {
            gymppError << "Failed to warm up the gazebo server in paused state" << std::endl;
            return nullptr;
        }
    }

    return gazebo.server;
}

// TODO: there's a bug in the destructor of sdf::Physics that prevents returning std::optional
bool GazeboWrapper::Impl::findAndLoadSdf(const std::string& sdfFileName, sdf::Root& root)
{
    if (sdfFileName.empty()) {
        gymppError << "The SDF file name of the gazebo model is empty" << std::endl;
        return {};
    }

    // Find the file
    // TODO: add install directory of our world and model files
    std::string sdfFilePath = systemPaths.FindFile(sdfFileName);

    if (sdfFilePath.empty()) {
        gymppError << "Failed to find '" << sdfFileName << "'. "
                   << "Check that it's contained in the paths defined in IGN_GAZEBO_RESOURCE_PATH."
                   << std::endl;
        return {};
    }

    // Load the sdf
    auto errors = root.Load(sdfFilePath);

    if (!errors.empty()) {
        gymppError << "Failed to load sdf file '" << sdfFilePath << "." << std::endl;
        for (const auto& error : errors) {
            gymppError << error << std::endl;
        }
        return {};
    }
    return true;
}

PhysicsData GazeboWrapper::Impl::getPhysicsData() const
{
    assert(sdf.WorldCount() == 1);
    assert(sdf.WorldByIndex(0)->PhysicsCount() == 1);

    PhysicsData physics;

    physics.rtf = sdf.WorldByIndex(0)->PhysicsByIndex(0)->RealTimeFactor();
    physics.maxStepSize = sdf.WorldByIndex(0)->PhysicsByIndex(0)->MaxStepSize();

    return physics;
}

bool GazeboWrapper::Impl::setPhysics(const PhysicsData& physicsData)
{
    assert(sdf.WorldCount() == 1);
    assert(sdf.WorldByIndex(0)->PhysicsCount() == 1);

    gymppDebug << "Setting physics profile" << std::endl;
    gymppDebug << "Desired RTF: " << physicsData.rtf << std::endl;
    gymppDebug << "Max physics step size:" << physicsData.maxStepSize << std::endl;
    gymppDebug << "Real time update rate: " << physicsData.realTimeUpdateRate << std::endl;

    if (physicsData.rtf <= 0) {
        gymppError << "The real time factor cannot be less than zero" << std::endl;
        return false;
    }

    if (physicsData.maxStepSize <= 0) {
        gymppError << "The maximum step size of the physics cannot be less than zero" << std::endl;
        return false;
    }

    // Set the physics properties using the helper.
    // This sets the internal value but it does not update the DOM.
    {
        auto* physics = const_cast<sdf::Physics*>(sdf.WorldByIndex(0)->PhysicsByIndex(0));
        physics->SetMaxStepSize(physicsData.maxStepSize);
        physics->SetRealTimeFactor(physicsData.rtf);
    }

    // Update the DOM operating directly on the raw elements
    {
        sdf::ElementPtr world = sdf.Element()->GetElement("world");
        assert(world);

        sdf::ElementPtr physics = world->GetElement("physics");
        assert(physics);

        sdf::ElementPtr max_step_size = physics->GetElement("max_step_size");
        max_step_size->AddValue("double", std::to_string(physicsData.maxStepSize), true);

        sdf::ElementPtr real_time_update_rate = physics->GetElement("real_time_update_rate");
        real_time_update_rate->AddValue(
            "double", std::to_string(physicsData.realTimeUpdateRate), true);

        sdf::ElementPtr real_time_factor = physics->GetElement("real_time_factor");
        real_time_factor->AddValue("double", std::to_string(physicsData.rtf), true);
    }

    assert(getPhysicsData() == gazebo.physics);
    return true;
}

// =============
// GAZEBOWRAPPER
// =============

GazeboWrapper::GazeboWrapper(const size_t numOfIterations,
                             const double desiredRTF,
                             const double physicsUpdateRate)
    : pImpl{new GazeboWrapper::Impl, [](Impl* impl) { delete impl; }}
{
    // Configure gazebo
    pImpl->gazebo.numOfIterations = numOfIterations;

    // Configure the physics profile
    pImpl->gazebo.physics.rtf = desiredRTF;
    pImpl->gazebo.physics.maxStepSize = 1 / physicsUpdateRate;

    if (numOfIterations == 0) {
        gymppError << "Failed to set the number of gazebo iterations" << std::endl;
        gymppError << "Check that the agent rate is not higher than the physics rate" << std::endl;
        assert(numOfIterations != 0);
    }

    // Configure search path of resources
    pImpl->systemPaths.SetFilePathEnv("IGN_GAZEBO_RESOURCE_PATH");
    pImpl->systemPaths.AddFilePaths(IGN_GAZEBO_WORLD_INSTALL_DIR);
    // TODO: add install world / models path

    // Set default verbosity which is dependent on the compilation flags
    setVerbosity();
}

GazeboWrapper::~GazeboWrapper()
{
    // Close the gui if it was opened
    close();
}

bool GazeboWrapper::initialize()
{
    // Initialize the server
    // TODO: check that all the other configures have been called
    if (!pImpl->getServer()) {
        gymppError << "Failed to get the ignition server" << std::endl;
        return false;
    }

    return true;
}

bool GazeboWrapper::run()
{
    // Get the gazebo server
    auto server = pImpl->getServer();
    if (!server) {
        gymppError << "Failed to get the ignition server" << std::endl;
        return false;
    }

    assert(server);
    assert(server->Paused());
    assert((server->Running() && server->Paused().value())
           || (!server->Running() && !server->Paused().value()));

    // Handle first iteration
    if (server->Running() && server->Paused().value()) {
        gymppDebug << "Unpausing the server. Running the first simulation run." << std::endl;
        bool ok = server->SetPaused(false);
        assert(ok);

        // Since the server was started in non-blocking mode, we have to wait that this first
        // iteration finishes
        while (server->Running()) {
            gymppDebug << "Waiting the first simulation run to finish..." << std::endl;
            assert(!server->Paused().value());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        gymppDebug << "First run ok" << std::endl;
    }
    // Run regular iteration
    else {
        if (!server->Run(/*blocking=*/true, pImpl->gazebo.numOfIterations, /*paused=*/false)) {
            gymppError << "The server couldn't execute the step" << std::endl;
            return false;
        }
    }

    return true;
}

bool GazeboWrapper::gui()
{
    // If ign-gazebo-gui is already running, return without doing anything.
    int exit_status;
    if (pImpl->gazebo.gui && !pImpl->gazebo.gui->try_get_exit_status(exit_status)) {
        return true;
    }

    // The GUI needs the ignition server running. Initialize it.
    if (!pImpl->getServer()) {
        gymppError << "Failed to get the ignition server" << std::endl;
        return false;
    }

    // Spawn a new process with the GUI
    pImpl->gazebo.gui = std::make_unique<TinyProcessLib::Process>("ign gazebo -g");

    return true;
}

bool GazeboWrapper::close()
{
    if (pImpl->gazebo.gui) {
#if defined(WIN32) || defined(_WIN32)
        bool force = false;
#else
        bool force = true;
#endif
        pImpl->gazebo.gui->kill(force);
    }

    return true;
}

PhysicsData GazeboWrapper::getPhysicsData() const
{
    return pImpl->gazebo.physics;
}

void GazeboWrapper::setVerbosity(int level)
{
    ignition::common::Console::SetVerbosity(level);
}

std::vector<GazeboWrapper::SdfModelName> GazeboWrapper::getModelNames() const
{
    return {pImpl->scopedModelName};
}

bool GazeboWrapper::setupGazeboModel(const std::string& modelFile, std::array<double, 6> /*pose*/)
{
    if (!pImpl->scopedModelName.empty()) {
        gymppError << "The model has been already configured previously" << std::endl;
        return false;
    }

    if (pImpl->sdf.WorldCount() <= 0) {
        gymppError << "The gazebo world was not properly configured" << std::endl;
        return false;
    }

    // Find and load the sdf file that contains the model
    sdf::Root sdfRoot;
    if (!pImpl->findAndLoadSdf(modelFile, sdfRoot)) {
        gymppError << "Failed to find and load sdf file '" << modelFile << "'" << std::endl;
        return false;
    }

    if (sdfRoot.ModelCount() == 0) {
        gymppError << "Failed to find any model in '" << modelFile << "' sdf file" << std::endl;
        return false;
    }

    // Get the models names included in the sdf file.
    // In order to allow multithreading, the name of the model contained in the sdf must be scoped.
    // We use the id of the IgnitionEnvironment object as scope.
    for (unsigned i = 0; i < sdfRoot.ModelCount(); ++i) {
        // Get the model name
        std::string modelName = sdfRoot.ModelByIndex(i)->Name();
        gymppDebug << "Found model '" << modelName << "' in the sdf file" << std::endl;

        // Create a scoped name
        std::string scopedModelName =
            std::to_string(reinterpret_cast<uint64_t>(this)) + "::" + modelName;
        gymppDebug << "Registering scoped '" << scopedModelName << "' model" << std::endl;

        // Copy the content of the model in another sdf element.
        // This has to be done because it is not possible to change in-place the name of the model,
        // so we create a new element and we copy all the children.
        sdf::ElementPtr renamedModel(new sdf::Element);
        renamedModel->SetName("model");
        renamedModel->AddAttribute("name", "string", scopedModelName, true);

        sdf::ElementPtr child = sdfRoot.ModelByIndex(i)->Element()->GetFirstElement();

        while (child) {
            renamedModel->InsertElement(child);
            child = child->GetNextElement();
        }

        // TODO: Temporarily support only one model, and it must be the one to which the gympp
        //       plugin will be attached to.
        assert(sdfRoot.ModelCount() == 1);
        pImpl->scopedModelName = scopedModelName;

        // Attach the model to the sdf world
        assert(pImpl->sdf.WorldCount() == 1);
        pImpl->sdf.WorldByIndex(0)->Element()->InsertElement(renamedModel);
    }

    return true;
}

bool GazeboWrapper::setupGazeboWorld(const std::string& worldFile)
{
    // Find and load the sdf file that contains the world
    if (!pImpl->findAndLoadSdf(worldFile, pImpl->sdf)) {
        gymppError << "Failed to find and load sdf file '" << worldFile << "'" << std::endl;
        return false;
    }

    if (!pImpl->setPhysics(pImpl->gazebo.physics)) {
        gymppError << "Failed to set physics profile" << std::endl;
        return false;
    }

    return true;
}

bool GazeboWrapper::setupIgnitionPlugin(const std::string& libName, const std::string& className)
{
    assert(!pImpl->scopedModelName.empty());

    // Create the plugin sdf element
    sdf::ElementPtr sdf(new sdf::Element);
    sdf->SetName("plugin");
    sdf->AddAttribute("name", "string", className, true);
    sdf->AddAttribute("filename", "string", libName, true);

    // Store the plugin information
    ignition::gazebo::ServerConfig::PluginInfo pluginInfo{
        pImpl->scopedModelName, "model", libName, className, sdf};

    // Insert the plugin context into the server configuration
    pImpl->gazebo.config.AddPlugin(pluginInfo);

    return true;
}
