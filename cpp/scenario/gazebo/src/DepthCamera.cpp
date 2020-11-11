/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "scenario/gazebo/sensors/DepthCamera.h"
#include "scenario/gazebo/Log.h"
#include "scenario/gazebo/components/DepthCameraPtr.h"
#include "scenario/gazebo/components/SensorsPlugin.h"
#include "scenario/gazebo/components/SimulatedTime.h"
#include "scenario/gazebo/helpers.h"

#include <ignition/common/events/Types.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/msgs/image.pb.h>
#include <ignition/sensors/DepthCameraSensor.hh>

#include <future>
#include <mutex>

using namespace scenario::gazebo::sensors;

class DepthCamera::Impl
{
public:
    mutable std::mutex mutex;
    ignition::common::ConnectionPtr connection = nullptr;
    ignition::sensors::DepthCameraSensor* depthCamera = nullptr;

    using SimTime = double;
    using ImageBuffer = std::vector<float>;
    using ImageBufferPtr = std::shared_ptr<ImageBuffer>;
    std::unordered_map<SimTime, std::promise<ImageBufferPtr>> promisedImages;

    ImageBufferPtr imageBuffer;
    mutable std::mutex copyImageDataCBMutex;

    ignition::common::Time t0;
    ignition::common::Time tNextImage;
    ignition::gazebo::Entity entityWithSimTime = ignition::gazebo::kNullEntity;

    static inline size_t ToNanoseconds(const ignition::common::Time& time)
    {
        return time.sec * 1e9 + time.nsec;
    }

    static inline size_t
    ToNanoseconds(const std::chrono::steady_clock::duration& time)
    {
        return std::chrono::nanoseconds(time).count();
    }
};

DepthCamera::DepthCamera()
    : pImpl{std::make_unique<Impl>()}
{}

DepthCamera::~DepthCamera() = default;

uint64_t DepthCamera::id() const
{
    // Get the parent world
    const core::WorldPtr parentWorld = utils::getParentWorld(*this);
    assert(parentWorld);

    // Build a unique string identifier of the sensors
    const std::string scopedSensorName =
        parentWorld->name() + "::" + this->name(/*scoped=*/true);

    // Return the hashed string
    return std::hash<std::string>{}(scopedSensorName);
}

bool DepthCamera::initialize(const ignition::gazebo::Entity parentEntity,
                             ignition::gazebo::EntityComponentManager* ecm,
                             ignition::gazebo::EventManager* eventManager)
{
    if (parentEntity == ignition::gazebo::kNullEntity || !ecm
        || !eventManager) {
        sError << "Failed to initialize DepthCamera" << std::endl;
        return false;
    }

    m_ecm = ecm;
    m_entity = parentEntity;
    m_eventManager = eventManager;

    const auto& parentWorld = utils::getParentWorld(*this);
    assert(parentWorld);

    // Fail if the Sensors system was not explicitly enabled
    if (!m_ecm->EntityHasComponentType(
            parentWorld->entity(),
            ignition::gazebo::components::SensorsPlugin::typeId)) {
        sError << "This world has the sensors system disabled" << std::endl;
        return false;
    }

    // Get the depth camera
    pImpl->depthCamera = utils::getExistingComponentData<
        ignition::gazebo::components::DepthCameraPtr>(ecm, parentEntity);

    // Configure the depth camera if the update rate is 0
    if (pImpl->depthCamera->UpdateRate()
        < std::numeric_limits<double>::epsilon()) {
        // TODO: do not hardcode the update rate
        const auto serverUpdateRate = 1000.;
        pImpl->depthCamera->SetUpdateRate(serverUpdateRate);
    }

    // Get the entity with simulated time
    pImpl->entityWithSimTime = utils::getFirstParentEntityWithComponent<
        ignition::gazebo::components::SimulatedTime>(m_ecm, m_entity);

    // Get the simulated time
    const auto& now = utils::steadyClockDurationToDouble(
        utils::getExistingComponentData<
            ignition::gazebo::components::SimulatedTime>(
            m_ecm, pImpl->entityWithSimTime));

    // Store the initial time. It is used to get the image at the time timestep.
    pImpl->t0 = ignition::common::Time(now);

    // Get the next time a new image is expected
    const ignition::common::Time dt(1.0 / pImpl->depthCamera->UpdateRate());
    pImpl->tNextImage = pImpl->t0 + dt;
    sDebug << "Image of camera " << this->name()
           << " availailable from t=" << pImpl->tNextImage.Double()
           << std::endl;

    // Allocate the buffer
    pImpl->imageBuffer = std::make_shared<Impl::ImageBuffer>();

    // Create the first promise and a dummy promise for the previous step.
    // Note: We use nanoseconds to avoid using floating point arithmetics
    //       on map's keys and being more robust to numerical errors.
    pImpl->promisedImages[Impl::ToNanoseconds(pImpl->tNextImage)] = {};
    pImpl->promisedImages[Impl::ToNanoseconds(pImpl->t0)].set_value(
        pImpl->imageBuffer);

    // Callback executed when a new image is available
    auto CopyImageDataCB = [&](const ignition::msgs::Image& img) -> void {
        // Execute the callback sequentially.
        // TODO: this does not guarantee the order of acquisition in case
        //       multiplet threads are waiting (in this case, we'd be interested
        //       only in the last one). For the moment we ignore this use case.
        //       A custom queued locking mechanism should be implemented.
        std::unique_lock lock(pImpl->copyImageDataCBMutex);

        // Get the time step of the camera
        const ignition::common::Time dtCamera(
            1.0 / pImpl->depthCamera->UpdateRate());

        // Get the sim time of the previous, current, and next images
        const auto tCurrentImage = pImpl->tNextImage;
        pImpl->tNextImage += dtCamera;
        const auto tPreviousImage = tCurrentImage - dtCamera;

        // Copy the image into the current buffer
        pImpl->imageBuffer->resize(img.width() * img.height());
        std::memcpy(pImpl->imageBuffer->data(),
                    img.data().c_str(),
                    pImpl->imageBuffer->size() * sizeof(float));

        // Store the current buffer into the current promise
        pImpl->promisedImages[Impl::ToNanoseconds(tCurrentImage)].set_value(
            pImpl->imageBuffer);

        // Create the next promise
        pImpl->promisedImages[Impl::ToNanoseconds(pImpl->tNextImage)] = {};

        // Delete the previous promise
        pImpl->promisedImages.erase(Impl::ToNanoseconds(tPreviousImage));
    };

    // Enable the callback
    pImpl->connection =
        pImpl->depthCamera->ConnectImageCallback(CopyImageDataCB);

    return true;
}

bool DepthCamera::createECMResources()
{
    sMessage << "  [" << m_entity << "] " << this->name() << std::endl;
    return true;
}

std::string DepthCamera::name(const bool scoped) const
{
    if (scoped) {
        return pImpl->depthCamera->Name();
    }

    const std::string scopedSensorName = pImpl->depthCamera->Name();
    const auto tokens = utils::tokenize(scopedSensorName, "::");
    assert(!tokens.empty());

    return tokens.back();
}

double DepthCamera::width() const
{
    return pImpl->depthCamera->RenderingCamera()->ImageWidth();
}

double DepthCamera::height() const
{
    return pImpl->depthCamera->RenderingCamera()->ImageHeight();
}

double DepthCamera::farClip() const
{
    return pImpl->depthCamera->FarClip();
}

double DepthCamera::nearClip() const
{
    return pImpl->depthCamera->NearClip();
}

const std::vector<float>& DepthCamera::image() const
{
    // Get the simulated time. It is used to retrieve the right image.
    const auto& tSim = utils::getExistingComponentData<
        ignition::gazebo::components::SimulatedTime //
        >(m_ecm, pImpl->entityWithSimTime);

    try {
        // Get the promise of this simulated time.
        // Raise if the promise does not exist.
        auto& promise = pImpl->promisedImages.at(Impl::ToNanoseconds(tSim));

        // Get the image buffer
        const auto& futureImage = promise.get_future().share();

        // Return the buffer
        return *futureImage.get();
    }
    catch (...) {
        const auto t = utils::steadyClockDurationToDouble(tSim);
        throw std::runtime_error("Failed to find image at t="
                                 + std::to_string(t));
    }
}

// bool DepthCamera::valid() const
//{
//    return this->validEntity() && pImpl->link.Valid(*m_ecm);
//}
