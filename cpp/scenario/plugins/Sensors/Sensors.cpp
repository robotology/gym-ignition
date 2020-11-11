/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "Sensors.h"
#include "scenario/gazebo/components/DepthCameraPtr.h"
#include "scenario/gazebo/components/SensorsPlugin.h"
#include "scenario/gazebo/helpers.h"

#include <ignition/common/Profiler.hh>
#include <ignition/common/Time.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/components/Atmosphere.hh>
#include <ignition/gazebo/components/Camera.hh>
#include <ignition/gazebo/components/DepthCamera.hh>
#include <ignition/gazebo/components/GpuLidar.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/RgbdCamera.hh>
#include <ignition/gazebo/components/ThermalCamera.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/rendering/RenderUtil.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/sensors/CameraSensor.hh>
#include <ignition/sensors/DepthCameraSensor.hh>
#include <ignition/sensors/Manager.hh>
#include <ignition/sensors/RenderingSensor.hh>
#include <ignition/sensors/ThermalCameraSensor.hh>
#include <sdf/Sensor.hh>

#include <set>

using namespace scenario::plugins::gazebo;

// Private data class.
class Sensors::SensorsPrivate
{
    /// \brief Sensor manager object. This manages the lifecycle of the
    /// instantiated sensors.
public:
    ignition::sensors::Manager sensorManager;

    /// \brief used to store whether rendering objects have been created.
public:
    bool initialized = false;

    /// \brief Main rendering interface
public:
    ignition::gazebo::RenderUtil renderUtil;

    /// \brief Unique set of sensor ids
    // TODO(anyone) Remove element when sensor is deleted
public:
    std::set<ignition::sensors::SensorId> sensorIds;

    /// \brief rendering scene to be managed by the scene manager and used to
    /// generate sensor data
public:
    ignition::rendering::ScenePtr scene;

    /// \brief Temperature used by thermal camera. Defaults to temperature at
    /// sea level
public:
    double ambientTemperature = 288.15;

    /// \brief Keep track of cameras, in case we need to handle stereo cameras.
    /// Key: Camera's parent scoped name
    /// Value: Pointer to camera
    // TODO(anyone) Remove element when sensor is deleted
public:
    std::map<std::string, ignition::sensors::CameraSensor*> cameras;

    /// \brief Flag to indicate if worker threads are running
public:
    std::atomic<bool> running{false};

    /// \brief Flag to signal if initialization should occur
public:
    bool doInit{false};

    /// \brief Flag to signal if rendering update is needed
public:
    bool updateAvailable{false};

    /// \brief Thread that rendering will occur in
public:
    std::thread renderThread;

    /// \brief Mutex to protect rendering data
public:
    std::mutex renderMutex;

    /// \brief Condition variable to signal rendering thread
    ///
    /// This variable is used to block/unblock operations in the rendering
    /// thread.  For a more detailed explanation on the flow refer to the
    /// documentation on RenderThread.
public:
    std::condition_variable renderCv;

    /// \brief Connection to events::Stop event, used to stop thread
public:
    ignition::common::ConnectionPtr stopConn;

    /// \brief Update time for the next rendering iteration
public:
    ignition::common::Time updateTime;

    /// \brief Sensors to include in the next rendering iteration
public:
    std::vector<ignition::sensors::RenderingSensor*> activeSensors;

    /// \brief Mutex to protect sensorMask
public:
    std::mutex sensorMaskMutex;

    /// \brief Mask sensor updates for sensors currently being rendered
public:
    std::map<ignition::sensors::SensorId, ignition::common::Time> sensorMask;

    /// \brief Marks whether PostUpdate is called from a paused or running state
public:
    bool pausedStep{false};

    /// \brief Wait for initialization to happen
private:
    void WaitForInit();

    /// \brief Run one rendering iteration
private:
    void RunOnce();

    /// \brief Top level function for the rendering thread
    ///
    /// This function captures all of the behavior of the rendering thread.
    /// The behavior is captured in two phases: initialization and steady state.
    ///
    /// When the thread is first started, it waits on renderCv until the
    /// prerequisites for initialization are met, and the `doInit` flag is set.
    /// In order for initialization to proceed, rendering sensors must be
    /// available in the EntityComponentManager.
    ///
    /// When doInit is set, and renderCv is notified, initialization
    /// is performed (creating the render context and scene). During
    /// initialization, execution is blocked for the caller of PostUpdate.
    /// When initialization is complete, PostUpdate will be notified via
    /// renderCv and execution will continue.
    ///
    /// Once in steady state, a rendering operation is triggered by setting
    /// updateAvailable to true, and notifying via the renderCv.
    /// The rendering operation is done in `RunOnce`.
    ///
    /// The caller of PostUpdate will not be blocked if there is no
    /// rendering operation currently ongoing. Rendering will occur
    /// asyncronously.
    //
    /// The caller of PostUpdate will be blocked if there is a rendering
    /// operation currently ongoing, until that completes.
private:
    void RenderThread();

public:
    /// Helper to store the pointer of a sensor in the ECM
    static bool
    StoreSensorIntoECM(ignition::sensors::Sensor* sensor,
                       const sdf::SensorType type,
                       const std::string& scopedParentName,
                       ignition::gazebo::EntityComponentManager& ecm);

public:
    /// Template for the helper that stores the pointer of a sensor in the ECM
    template <typename IgnitionSensorType,
              typename ComponentOfSensor,
              typename ComponentOfSensorPtr>
    static bool
    StoreSensorIntoECMTemplate(ignition::sensors::Sensor* sensor,
                               const std::string& scopedParentName,
                               ignition::gazebo::EntityComponentManager& ecm);

public:
    /// Helper to extract the entity and name of server from the ECM given
    /// its parent's scoped name.
    template <typename ComponentOfSensor>
    static bool
    GetSensorDataFromECM(ignition::gazebo::EntityComponentManager& ecm,
                         const std::string& scopedParentName,
                         std::string& sensorName,
                         ignition::gazebo::Entity& sensorEntity);

    /// \brief Launch the rendering thread
public:
    void Run();

    /// \brief Stop the rendering thread
public:
    void Stop();
};

//////////////////////////////////////////////////
void Sensors::SensorsPrivate::WaitForInit()
{
    while (!this->initialized && this->running) {
        ignwarn << "Waiting for init" << std::endl;
        std::unique_lock<std::mutex> lock(this->renderMutex);
        // Wait to be ready for initialization or stopped running.
        // We need rendering sensors to be available to initialize.
        this->renderCv.wait(
            lock, [this]() { return this->doInit || !this->running; });

        if (this->doInit) {
            // Only initialize if there are rendering sensors
            ignwarn << "Intializing render context" << std::endl;
            this->renderUtil.Init();
            this->scene = this->renderUtil.Scene();
            this->initialized = true;
        }

        this->updateAvailable = false;
        this->renderCv.notify_one(); // Unlock PostUpdate
    }
    ignwarn << "Rendering Thread initialized" << std::endl;
}

//////////////////////////////////////////////////
void Sensors::SensorsPrivate::RunOnce()
{
    std::unique_lock lock(this->renderMutex);
    this->renderCv.wait(
        lock, [this]() { return !this->running || this->updateAvailable; });

    if (!this->running)
        return;

    IGN_PROFILE("SensorsPrivate::RunOnce");
    {
        IGN_PROFILE("Update");
        this->renderUtil.Update();
    }

    if (!this->activeSensors.empty()) {
        this->sensorMaskMutex.lock();
        // Check the active sensors against masked sensors.
        //
        // The internal state of a rendering sensor is not updated until the
        // rendering operation is complete, which can leave us in a position
        // where the sensor is falsely indicating that an update is needed.
        //
        // To prevent this, add sensors that are currently being rendered to
        // a mask. Sensors are removed from the mask when 90% of the update
        // delta has passed, which will allow rendering to proceed.
        for (const auto& sensor : this->activeSensors) {
            // 90% of update delta (1/UpdateRate());
            ignition::common::Time delta(0.9 / sensor->UpdateRate());
            // this->sensorMask[sensor->Id()] = this->updateTime + delta;
        }
        this->sensorMaskMutex.unlock();

        {
            IGN_PROFILE("PreRender");
            // Update the scene graph manually to improve performance
            // We only need to do this once per frame It is important to call
            // sensors::RenderingSensor::SetManualSceneUpdate and set it to true
            // so we don't waste cycles doing one scene graph update per sensor
            this->scene->PreRender();
        }

        if (!pausedStep) {
            // publish data
            IGN_PROFILE("RunOnce");
            this->sensorManager.RunOnce(this->updateTime);
        }

        this->activeSensors.clear();
    }

    this->updateAvailable = false;
    lock.unlock();
    this->renderCv.notify_one();
}

//////////////////////////////////////////////////
void Sensors::SensorsPrivate::RenderThread()
{
    IGN_PROFILE_THREAD_NAME("RenderThread");

    ignwarn << "SensorsPrivate::RenderThread started" << std::endl;

    // We have to wait for rendering sensors to be available
    this->WaitForInit();

    while (this->running) {
        this->RunOnce();
    }

    for (auto id : this->sensorIds)
        this->sensorManager.Remove(id);

    ignwarn << "SensorsPrivate::RenderThread stopped" << std::endl;
}

//////////////////////////////////////////////////
void Sensors::SensorsPrivate::Run()
{
    ignwarn << "SensorsPrivate::Run" << std::endl;
    this->running = true;
    this->renderThread = std::thread(&SensorsPrivate::RenderThread, this);
}

//////////////////////////////////////////////////
void Sensors::SensorsPrivate::Stop()
{
    ignwarn << "SensorsPrivate::Stop" << std::endl;
    std::unique_lock<std::mutex> lock(this->renderMutex);
    this->running = false;

    if (this->stopConn) {
        // Clear connection to stop additional incoming events.
        this->stopConn.reset();
    }

    lock.unlock();
    this->renderCv.notify_all();

    if (this->renderThread.joinable()) {
        this->renderThread.join();
    }
}

//////////////////////////////////////////////////
Sensors::Sensors()
    : System()
    , dataPtr(std::make_unique<SensorsPrivate>())
{}

//////////////////////////////////////////////////
Sensors::~Sensors()
{
    this->dataPtr->Stop();
}

//////////////////////////////////////////////////
void Sensors::Configure(const ignition::gazebo::Entity& _entity,
                        const std::shared_ptr<const sdf::Element>& _sdf,
                        ignition::gazebo::EntityComponentManager& _ecm,
                        ignition::gazebo::EventManager& _eventMgr)
{
    ignwarn << "Configuring Sensors system" << std::endl;

    // Check if the model already has a Sensors plugin
    if (_ecm.EntityHasComponentType(
            _entity, ignition::gazebo::components::SensorsPlugin::typeId)) {
        ignerr << "The world already has a Sensors plugin" << std::endl;
        return;
    }

    // Setup rendering
    std::string engineName =
        _sdf->Get<std::string>("render_engine", "ogre2").first;

    // Close the CreateSensor method in a lambda to catch the ECM required to
    // store the sensor pointers in custom components
    auto CreateSensorCB = [&](const sdf::Sensor& sdf,
                              const std::string& parentName) -> std::string {
        return this->CreateSensor(sdf, parentName, _ecm);
    };

    this->dataPtr->renderUtil.SetEngineName(engineName);
    this->dataPtr->renderUtil.SetEnableSensors(true, CreateSensorCB);

    // parse sensor-specific data
    auto worldEntity =
        _ecm.EntityByComponents(ignition::gazebo::components::World());
    if (ignition::gazebo::kNullEntity != worldEntity) {
        // temperature used by thermal camera
        auto atmosphere =
            _ecm.Component<ignition::gazebo::components::Atmosphere>(
                worldEntity);
        if (atmosphere) {
            auto atmosphereSdf = atmosphere->Data();
            this->dataPtr->ambientTemperature =
                atmosphereSdf.Temperature().Kelvin();
        }
    }

    this->dataPtr->stopConn = _eventMgr.Connect<ignition::gazebo::events::Stop>(
        std::bind(&SensorsPrivate::Stop, this->dataPtr.get()));

    // Kick off worker thread
    this->dataPtr->Run();

    // Add the SensorsPlugin component to the world
    scenario::gazebo::utils::setComponentData<
        ignition::gazebo::components::SensorsPlugin>(&_ecm, _entity, true);
}

//////////////////////////////////////////////////
void Sensors::PostUpdate(const ignition::gazebo::UpdateInfo& _info,
                         const ignition::gazebo::EntityComponentManager& _ecm)
{
    IGN_PROFILE("Sensors::PostUpdate");

    // \TODO(anyone) Support rewind
    if (_info.dt < std::chrono::steady_clock::duration::zero()) {
        ignwarn << "Detected jump back in time ["
                << std::chrono::duration_cast<std::chrono::seconds>(_info.dt)
                       .count()
                << "s]. System may not work properly." << std::endl;
    }

    // Set the state of the simulation. Paused step call Scene::Prerender() but
    // do not execute Manager::RunOnce(). This prevents sensors to generate data
    // and mess with the time handling of the classes that extract data from the
    // ECM exposing C++ APIs.
    this->dataPtr->pausedStep = _info.paused;

    if (!this->dataPtr->initialized
        && (_ecm.HasComponentType(ignition::gazebo::components::Camera::typeId)
            || _ecm.HasComponentType(
                ignition::gazebo::components::DepthCamera::typeId)
            || _ecm.HasComponentType(
                ignition::gazebo::components::GpuLidar::typeId)
            || _ecm.HasComponentType(
                ignition::gazebo::components::RgbdCamera::typeId)
            || _ecm.HasComponentType(
                ignition::gazebo::components::ThermalCamera::typeId))) {
        {
            igndbg << "Initialization needed" << std::endl;
            std::unique_lock lock(this->dataPtr->renderMutex);
            this->dataPtr->doInit = true;
            this->dataPtr->renderCv.notify_one(); // Unlock WaitForInit
        }

        {
            // Wait the render context to be ready
            std::unique_lock lock(this->dataPtr->renderMutex);
            this->dataPtr->renderCv.wait(lock, [this] {
                return this->dataPtr->running && this->dataPtr->initialized;
            });
        }
    }

    if (this->dataPtr->running && this->dataPtr->initialized) {
        this->dataPtr->renderUtil.UpdateFromECM(_info, _ecm);

        auto time = ignition::math::durationToSecNsec(_info.simTime);
        auto t = ignition::common::Time(time.first, time.second);

        std::vector<ignition::sensors::RenderingSensor*> activeSensors;

        this->dataPtr->sensorMaskMutex.lock();
        for (auto id : this->dataPtr->sensorIds) {
            ignition::sensors::Sensor* s =
                this->dataPtr->sensorManager.Sensor(id);
            auto rs = dynamic_cast<ignition::sensors::RenderingSensor*>(s);

            auto it = this->dataPtr->sensorMask.find(id);
            if (it != this->dataPtr->sensorMask.end()) {
                if (it->second <= t) {
                    // second = +inf -> never happens
                    this->dataPtr->sensorMask.erase(it);
                }
                else {
                    continue;
                }
            }

            if (rs && rs->NextUpdateTime() <= t) {
                activeSensors.push_back(rs);
            }
        }
        this->dataPtr->sensorMaskMutex.unlock();

        if (this->dataPtr->running
            && (!activeSensors.empty()
                || this->dataPtr->renderUtil.PendingSensors() > 0)) {

            this->dataPtr->activeSensors = std::move(activeSensors);
            this->dataPtr->updateTime = t;
            this->dataPtr->updateAvailable = true;

            // Trigger RunOnce to create the new sensors and render
            // the other ones
            this->dataPtr->renderCv.notify_one();

            // Wait that RunOnce finishes. Note that this does not assure that
            // sensors finished to generate their data. The callbacks that can
            // be associated to each sensor to get their data must properly
            // handle the resulting asynchronous behaviour.
            std::unique_lock lock(this->dataPtr->renderMutex);
            this->dataPtr->renderCv.wait(
                lock, [this] { return !this->dataPtr->updateAvailable; });
        }
    }
}

//////////////////////////////////////////////////
std::string
Sensors::CreateSensor(const sdf::Sensor& _sdf,
                      const std::string& _parentName,
                      ignition::gazebo::EntityComponentManager& _ecm)
{
    if (_sdf.Type() == sdf::SensorType::NONE) {
        ignerr << "Unable to create sensor. SDF sensor type is NONE."
               << std::endl;
        return std::string();
    }

    // Create within ign-sensors
    auto sensorId = this->dataPtr->sensorManager.CreateSensor(_sdf);
    auto sensor = this->dataPtr->sensorManager.Sensor(sensorId);

    if (nullptr == sensor || ignition::sensors::NO_SENSOR == sensor->Id()) {
        ignerr << "Failed to create sensor [" << _sdf.Name() << "]"
               << std::endl;
        return std::string();
    }

    this->dataPtr->sensorIds.insert(sensorId);

    // Set the scene so it can create the rendering sensor
    auto renderingSensor =
        dynamic_cast<ignition::sensors::RenderingSensor*>(sensor);
    renderingSensor->SetScene(this->dataPtr->scene);
    renderingSensor->SetParent(_parentName);
    renderingSensor->SetManualSceneUpdate(true);

    // Special case for stereo cameras
    auto cameraSensor = dynamic_cast<ignition::sensors::CameraSensor*>(sensor);
    if (nullptr != cameraSensor) {
        // Parent
        auto parent = cameraSensor->Parent();

        // If parent has other camera children, set the baseline.
        // For stereo pairs, the baseline for the left camera is zero, and for
        // the right camera it's the distance between them. For more than 2
        // cameras, the first camera's baseline is zero and the others have the
        // distance between them.
        if (this->dataPtr->cameras.find(parent)
            != this->dataPtr->cameras.end()) {
            // TODO(anyone) This is safe because we're not removing sensors
            // First camera added to the parent link
            auto leftCamera = this->dataPtr->cameras[parent];
            auto rightCamera = cameraSensor;

            // If cameras have right / left topic, use that to decide which is
            // which
            if (leftCamera->Topic().find("right") != std::string::npos
                && rightCamera->Topic().find("left") != std::string::npos) {
                std::swap(rightCamera, leftCamera);
            }

            // Camera sensor's Y axis is orthogonal to the optical axis
            auto baseline = abs(rightCamera->Pose().Pos().Y()
                                - leftCamera->Pose().Pos().Y());
            rightCamera->SetBaseline(baseline);
        }
        else {
            this->dataPtr->cameras[parent] = cameraSensor;
        }
    }

    // Sensor-specific settings
    auto thermalSensor =
        dynamic_cast<ignition::sensors::ThermalCameraSensor*>(sensor);
    if (nullptr != thermalSensor) {
        thermalSensor->SetAmbientTemperature(this->dataPtr->ambientTemperature);
    }

    // Store the sensor in the ECM to access it without relying on transport
    if (!Sensors::SensorsPrivate::StoreSensorIntoECM(
            sensor, _sdf.Type(), _parentName, _ecm)) {
        ignerr << "Failed to store sensor '" << sensor->Name() << "' in the ECM"
               << std::endl;
    }

    return sensor->Name();
}

//////////////////////////////////////////////////
template <typename ComponentOfSensor>
bool Sensors::SensorsPrivate::GetSensorDataFromECM(
    ignition::gazebo::EntityComponentManager& ecm,
    const std::string& scopedParentName,
    std::string& sensorName,
    ignition::gazebo::Entity& sensorEntity)
{
    // Tokenize the scoped parent name into a vector [modelName, linkName]
    const auto tokens =
        scenario::gazebo::utils::tokenize(scopedParentName, "::");

    if (tokens.size() != 2) {
        ignerr << "Failed to tokenize scoped name of sensor's parent"
               << std::endl;
        return false;
    }

    const std::string& linkName = tokens[1];
    const std::string& modelName = tokens[0];

    // Get the name and the entity of the sensor
    ecm.Each<ignition::gazebo::components::Name,
             ignition::gazebo::components::ParentEntity,
             ComponentOfSensor>(
        [&](const ignition::gazebo::Entity& entity,
            ignition::gazebo::components::Name* nameComponent,
            ignition::gazebo::components::ParentEntity* parentEntityComponent,
            ComponentOfSensor*) -> bool {
            assert(nameComponent);
            assert(parentEntityComponent);

            const auto parentEntity = parentEntityComponent->Data();
            const auto grandparentEntity = ecm.ParentEntity(parentEntity);

            if (parentEntity == ignition::gazebo::kNullEntity
                || grandparentEntity == ignition::gazebo::kNullEntity) {
                return true;
            }

            const auto& parentEntityName =
                scenario::gazebo::utils::getExistingComponentData<
                    ignition::gazebo::components::Name>(&ecm, parentEntity);

            const auto& grandparentEntityName =
                scenario::gazebo::utils::getExistingComponentData<
                    ignition::gazebo::components::Name>(&ecm,
                                                        grandparentEntity);

            if (parentEntityName == linkName
                && grandparentEntityName == modelName) {
                sensorEntity = entity;
                sensorName = nameComponent->Data();
            }

            return true;
        });

    if (sensorEntity == ignition::gazebo::kNullEntity || sensorName.empty()) {
        ignerr << "Failed to process sensor. "
               << "Couldn't find the entity with scoped name "
               << scopedParentName << std::endl;
        return false;
    }

    return true;
}

//////////////////////////////////////////////////
template <typename IgnitionSensorType,
          typename ComponentOfSensor,
          typename ComponentOfSensorPtr>
bool Sensors::SensorsPrivate::StoreSensorIntoECMTemplate(
    ignition::sensors::Sensor* sensor,
    const std::string& scopedParentName,
    ignition::gazebo::EntityComponentManager& ecm)
{
    // Downcast the sensor and check if the type is correct
    auto downcastSensor = dynamic_cast<IgnitionSensorType*>(sensor);

    if (!downcastSensor) {
        ignerr << "Failed to downcast sensor" << std::endl;
        return false;
    }

    std::string sensorName;
    ignition::gazebo::Entity sensorEntity;

    // Extract the name and entity of the sensor
    if (!SensorsPrivate::GetSensorDataFromECM<ComponentOfSensor>(
            ecm, scopedParentName, sensorName, sensorEntity)) {
        ignerr << "Failed to get the name and entity of the sensor "
               << "from the ECM" << std::endl;
        return false;
    }

    // Store the downcasted pointer in the ECM. It is used by ScenarI/O
    // to programmatically extract data from C++.
    scenario::gazebo::utils::setComponentData<ComponentOfSensorPtr>(
        &ecm, sensorEntity, downcastSensor);

    ignmsg << "Successfully processed sensor '"
           << scopedParentName + "::" + sensorName << "' (id=" << sensor->Id()
           << ")" << std::endl;
    return true;
}

//////////////////////////////////////////////////
bool Sensors::SensorsPrivate::StoreSensorIntoECM(
    ignition::sensors::Sensor* sensor,
    const sdf::SensorType type,
    const std::string& scopedParentName,
    ignition::gazebo::EntityComponentManager& ecm)
{
    switch (type) {
        case sdf::SensorType::DEPTH_CAMERA: {
            if (!SensorsPrivate::StoreSensorIntoECMTemplate<
                    ignition::sensors::DepthCameraSensor,
                    ignition::gazebo::components::DepthCamera,
                    ignition::gazebo::components::DepthCameraPtr>(
                    sensor, scopedParentName, ecm)) {
                ignerr << "Failed to store the sensor in the ECM" << std::endl;
                return false;
            }
            break;
        }
        default:
            ignerr << "Sensor type " << int(type) << " not yet supported";
            return false;
    }

    return true;
}

IGNITION_ADD_PLUGIN(scenario::plugins::gazebo::Sensors,
                    scenario::plugins::gazebo::Sensors::System,
                    scenario::plugins::gazebo::Sensors::ISystemConfigure,
                    scenario::plugins::gazebo::Sensors::ISystemPostUpdate)
