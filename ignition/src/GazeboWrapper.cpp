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
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <cassert>
#include <chrono>

using namespace gympp::gazebo;

size_t GazeboWrapper::EnvironmentId = 0;

struct GazeboData
{
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

    ignition::common::SystemPaths systemPaths;

    std::string scopedModelName;
};

std::shared_ptr<ignition::gazebo::Server> GazeboWrapper::Impl::getServer()
{
    // Lazy initialization of the server
    if (!gazebo.server) {

        assert(sdf.Element());
        assert(!sdf.Element()->ToString("").empty());
        gazebo.config.SetSdfString(sdf.Element()->ToString(""));

        sdf::Root root;
        auto errors = root.LoadSdfString(sdf.Element()->ToString(""));
        assert(errors.empty()); // This should be already ok

        // Create the server
        gymppDebug << "Creating the server" << std::endl << std::flush;
        gazebo.config.SetUseLevels(false);
        gazebo.server = std::make_shared<ignition::gazebo::Server>(gazebo.config);
        assert(gazebo.server);

        // The GUI needs the server already up. Warming up the first iteration and pausing the
        // server. It will be unpaused at the step() call.
        gymppDebug << "Starting the server as paused" << std::endl;
        gymppDebug << "Running " << gazebo.numOfIterations << " iterations every step" << std::endl;
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

GazeboWrapper::GazeboWrapper(size_t updateRate, uint64_t iterations)
    : pImpl{new GazeboWrapper::Impl, [](Impl* impl) { delete impl; }}
{
    // Assign an unique id to the object
    pImpl->id = EnvironmentId++;

    // Configure gazebo
    pImpl->gazebo.numOfIterations = iterations;
    pImpl->gazebo.config.SetUpdateRate(updateRate);

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

    // Handle first iteration
    if (server->Running()) {
        gymppDebug << "Unpausing the server. Running the first simulation run." << std::endl;
        server->SetPaused(false);

        // Since the server was started in non-blocking mode, we have to wait that this first
        // iteration finishes
        while (server->Running()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
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
    pImpl->gazebo.gui = std::make_unique<TinyProcessLib::Process>("ign-gazebo-gui");

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

double GazeboWrapper::getUpdateRate() const
{
    auto updateRate = pImpl->gazebo.config.UpdateRate();
    assert(updateRate);
    return updateRate.value();
}

uint64_t GazeboWrapper::getNumberOfIterations() const
{
    return pImpl->gazebo.numOfIterations;
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
        std::string scopedModelName = std::to_string(pImpl->id) + "::" + modelName;
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

    return true;
}

bool GazeboWrapper::setupIgnitionPlugin(const std::string& libName,
                                        const std::string& className,
                                        size_t updateRate)
{
    assert(!pImpl->scopedModelName.empty());

    sdf::ElementPtr sdf(new sdf::Element);
    sdf->SetName("plugin");
    sdf->AddAttribute("name", "string", className, true);
    sdf->AddAttribute("filename", "string", libName, true);

    // Set the update rate of the plugin. By default the update rate of the physic engine is used.
    if (updateRate != 0.0) {
        // Rate of the agent
        auto element = std::make_shared<sdf::Element>();
        element->SetName("update_rate");
        element->AddValue("double", std::to_string(updateRate), true);
        sdf->InsertElement(element);
    }

    ignition::gazebo::ServerConfig::PluginInfo pluginInfo{
        pImpl->scopedModelName, "model", libName, className, sdf};

    pImpl->gazebo.config.AddPlugin(pluginInfo);

    // Update the number of iterations accordingly to the simulation step and plugin update rate
    assert(pImpl->gazebo.config.UpdateRate());
    pImpl->gazebo.numOfIterations =
        static_cast<size_t>(pImpl->gazebo.config.UpdateRate().value() / updateRate);

    if ((static_cast<size_t>(pImpl->gazebo.config.UpdateRate().value()) % updateRate) != 0) {
        gymppWarning << "Rounding the number of iterations to " << pImpl->gazebo.numOfIterations
                     << " from the nominal "
                     << pImpl->gazebo.config.UpdateRate().value() / updateRate << std::endl;
    }

    return true;
}
