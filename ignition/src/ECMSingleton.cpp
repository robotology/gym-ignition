/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/gazebo/ECMSingleton.h"
#include "gympp/Log.h"

#include <mutex>
#include <ostream>
#include <unordered_map>

using namespace gympp::gazebo;
using RobotName = std::string;

struct Pointers
{
    std::atomic<ignition::gazebo::EventManager*> eventMgr = nullptr;
    std::atomic<ignition::gazebo::EntityComponentManager*> ecm = nullptr;
};

struct PreUpdateSynchronizationData
{
    bool isPreUpdate;
    std::mutex preUpdateMutex;
    std::mutex processingMutex;
    std::condition_variable executorsCV;
    std::condition_variable preUpdateCV;
    std::atomic<size_t> nrExecutorsWaiting;
};

class ECMSingleton::Impl
{
public:
    using WorldName = std::string;
    std::unordered_map<WorldName, Pointers> resources;
    std::unordered_map<WorldName, PreUpdateSynchronizationData> synchronizationData;
};

ECMSingleton::ECMSingleton()
    : pImpl{new Impl(), [](Impl* impl) { delete impl; }}
{}

ECMSingleton& ECMSingleton::get()
{
    static ECMSingleton instance;
    return instance;
}

bool ECMSingleton::valid(const std::string& worldName) const
{
    return exist(worldName) && pImpl->resources.at(worldName).ecm
           && pImpl->resources.at(worldName).eventMgr;
}

bool ECMSingleton::exist(const std::string& worldName) const
{
    if (pImpl->resources.find(worldName) != pImpl->resources.end()) {
        return true;
    }
    else {
        return false;
    }
}

ignition::gazebo::EventManager* ECMSingleton::getEventManager(const std::string& worldName) const
{
    if (!exist(worldName)) {
        gymppError << "The event manager was never stored" << std::endl;
        return nullptr;
    }

    if (!valid(worldName)) {
        gymppError << "The pointers are not valid" << std::endl;
        return nullptr;
    }

    return pImpl->resources.at(worldName).eventMgr;
}

bool ECMSingleton::storePtrs(const std::string& worldName,
                             ignition::gazebo::EntityComponentManager* ecm,
                             ignition::gazebo::EventManager* eventMgr)
{
    if (!ecm || !eventMgr) {
        gymppError << "The pointer to the ECM or EventManager is null" << std::endl;
        return false;
    }

    if (exist(worldName)) {
        gymppWarning << "The pointers for world '" << worldName << "' have already been stored."
                     << " This method will do nothing" << std::endl;
        return true;
    }

    gymppDebug << "Storing the ECM and the EventManager in the singleton" << std::endl;
    pImpl->resources[worldName].ecm = ecm;
    pImpl->resources[worldName].eventMgr = eventMgr;

    return true;
}

void ECMSingleton::notifyExecutorFinished(const std::string& worldName)
{
    pImpl->synchronizationData[worldName].executorsCV.notify_all();
}

void ECMSingleton::notifyAndWaitPreUpdate(const std::string& worldName)
{
    auto& syncroData = pImpl->synchronizationData[worldName];

    // Temporarily lock the executors mutex (see the waitPreUpdate method).
    std::unique_lock processingLock(syncroData.processingMutex);

    // Notify to all executors that the simulation reached the PreUpdate step
    syncroData.isPreUpdate = true;
    syncroData.preUpdateCV.notify_all();

    // If the are pending executors, they are waiting in their condition variable lambda.
    // Unlock the executors mutex and wait that all executors are processed.
    syncroData.executorsCV.wait(processingLock, [&] { return syncroData.nrExecutorsWaiting == 0; });
    syncroData.isPreUpdate = false;
}

std::unique_lock<std::mutex> ECMSingleton::waitPreUpdate(const std::string& worldName)
{
    // Let's define 'executor' every function that calls this method.
    // Executors want to wait to reach the PreUpdate step of the simulation, block its execution,
    // and run some custom code (e.g. adding entities in the ECM).
    // Executors run syncronously, one after the other. The simulation starts again when all
    // executors finished.

    auto& syncroData = pImpl->synchronizationData[worldName];

    // Initialize the lock returned to the executor to wait for their processing
    std::unique_lock processingLock(syncroData.processingMutex, std::defer_lock);

    // Executors can arrive here in parallel. When this variable is back to 0, the simulation
    // starts again.
    syncroData.nrExecutorsWaiting++;

    // Process one executor at time and wait to reach the PreUpdate step
    std::unique_lock lock(syncroData.preUpdateMutex);
    syncroData.preUpdateCV.wait(lock, [&] {
        if (syncroData.isPreUpdate) {
            // When the simulation reaches the PreUpdate step, this lambda is called.
            // One executors acquires the lock and continues. The others are blocked here.
            processingLock.lock();
            return true;
        }
        return false;
    });

    // Decrease the counter
    syncroData.nrExecutorsWaiting--;

    // Return the lock. When it goes out of executor scope, the next executors (if any) will get
    // unlocked from the condition variable lambda.
    return processingLock;
}

ignition::gazebo::EntityComponentManager* ECMSingleton::getECM(const std::string& worldName) const
{
    if (!exist(worldName)) {
        gymppError << "The ECM of world '" << worldName << "' was never stored" << std::endl;
        return nullptr;
    }

    if (!valid(worldName)) {
        gymppError << "The pointers are not valid" << std::endl;
        return nullptr;
    }

    return pImpl->resources.at(worldName).ecm;
}

void ECMSingleton::clean(const std::string& worldName)
{
    pImpl->resources.erase(worldName);
    pImpl->synchronizationData.erase(worldName);
}
