/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_METADATA
#define GYMPP_METADATA

#include "gympp/Log.h"
#include "gympp/Space.h"
#include "gympp/gazebo/IgnitionEnvironment.h"

#include <string>
#include <vector>

namespace gympp {
    class GymFactory;
    class SpaceMetadata;
    class PluginMetadata;

    enum class SpaceType
    {
        Discrete,
        Box
    };
} // namespace gympp

class gympp::SpaceMetadata
{
private:
    friend gympp::GymFactory;

    SpaceType type;
    std::vector<size_t> dims;

    gympp::spaces::Box::Limit low;
    gympp::spaces::Box::Limit high;

    bool boxSpaceValid() const
    {
        if (low.size() != high.size()) {
            gymppError << "The size of the limits do not match" << std::endl;
            return false;
        }

        if (dims.empty()) {
            if (low.empty()) {
                gymppError << "The limits do not contain any data" << std::endl;
                return false;
            }
        }
        else {
            if (low.size() != 1) {
                gymppError << "The limits must be scalar values" << std::endl;
                return false;
            }
        }

        return true;
    }

    bool discreteSpaceValid() const
    {
        if (dims.size() != 1 && dims[0] <= 0) {
            return false;
        }

        return true;
    }

public:
    inline SpaceType getType() const { return type; }
    inline std::vector<size_t> getDimensions() const { return dims; }
    inline gympp::spaces::Box::Limit getLowLimit() const { return low; }
    inline gympp::spaces::Box::Limit getHighLimit() const { return high; }

    inline void setType(const SpaceType type) { this->type = type; }
    inline void setDimensions(const std::vector<size_t>& dims) { this->dims = dims; }
    inline void setLowLimit(const gympp::spaces::Box::Limit& limit) { this->low = limit; }
    inline void setHighLimit(const gympp::spaces::Box::Limit& limit) { this->high = limit; }

    bool isValid() const
    {
        switch (type) {
            case SpaceType::Box:
                return boxSpaceValid();
            case SpaceType::Discrete:
                return discreteSpaceValid();
        }

        return true;
    }
};

class gympp::PluginMetadata
{
private:
    friend gympp::GymFactory;
    std::string environmentName;
    std::string libraryName;
    std::string className;

    std::string modelFileName;
    std::string worldFileName;

    double agentRate;
    gazebo::PhysicsData physicsData;

    SpaceMetadata actionSpace;
    SpaceMetadata observationSpace;

public:
    inline std::string getEnvironmentName() const { return environmentName; }
    inline std::string getLibraryName() const { return libraryName; }
    inline std::string getClassName() const { return className; }
    inline std::string getModelFileName() const { return modelFileName; }
    inline std::string getWorldFileName() const { return worldFileName; }
    inline double getAgentRate() const { return agentRate; }
    inline gazebo::PhysicsData getPhysicsData() const { return physicsData; }
    inline SpaceMetadata getActionSpaceMetadata() const { return actionSpace; }
    inline SpaceMetadata getObservationSpaceMetadata() const { return observationSpace; }

    inline void setEnvironmentName(const std::string& environmentName)
    {
        this->environmentName = environmentName;
    }

    inline void setActionSpaceMetadata(const SpaceMetadata& actionSpaceMetadata)
    {
        this->actionSpace = actionSpaceMetadata;
    }

    inline void setObservationSpaceMetadata(const SpaceMetadata& observationSpaceMetadata)
    {
        this->observationSpace = observationSpaceMetadata;
    }

    inline void setLibraryName(const std::string& libraryName) { this->libraryName = libraryName; }
    inline void setClassName(const std::string& className) { this->className = className; }
    inline void setModelFileName(const std::string& modelFileName)
    {
        this->modelFileName = modelFileName;
    }
    inline void setWorldFileName(const std::string& worldFileName)
    {
        this->worldFileName = worldFileName;
    }

    inline void setAgentRate(const double agentRate) { this->agentRate = agentRate; }

    inline void setPhysicsData(const gazebo::PhysicsData& physicsData)
    {
        this->physicsData = physicsData;
    }

    bool isValid() const
    {
        bool ok = true;
        ok = ok && !environmentName.empty();
        ok = ok && !libraryName.empty();
        ok = ok && !className.empty();
        ok = ok && !modelFileName.empty();
        ok = ok && !worldFileName.empty();
        ok = ok && actionSpace.isValid();
        ok = ok && observationSpace.isValid();
        ok = ok && physicsData.rtf > 0;
        ok = ok && physicsData.maxStepSize > 0;
        ok = ok && agentRate > 0;
        return ok;
    }
};

#endif // GYMPP_METADATA
