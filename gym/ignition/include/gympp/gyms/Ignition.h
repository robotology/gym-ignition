#ifndef GYMPP_GYMS_IGNITION
#define GYMPP_GYMS_IGNITION

#include "gympp/Gympp.h"

#include <memory>

namespace gympp {
    namespace gyms {
        class IgnitionGazebo;
        class EnvironmentBehavior;
    } // namespace gyms
} // namespace gympp

// TODO
namespace ignition {
    namespace gazebo {
        inline namespace v0 {
            class Server;
        } // namespace v0
    } // namespace gazebo
} // namespace ignition

class gympp::gyms::EnvironmentBehavior
{
public:
    using Action = gympp::Environment::Action;
    using Observation = gympp::Environment::Observation;
    using Reward = gympp::Environment::Reward;

    virtual ~EnvironmentBehavior() = default;

    virtual bool isDone() = 0;
    virtual bool reset() = 0;
    virtual bool setAction(const Action& action) = 0;
    virtual std::optional<Reward> computeReward() = 0;
    virtual std::optional<Observation> getObservation() = 0;
};

class gympp::gyms::IgnitionGazebo
    : public gympp::Environment
    , public std::enable_shared_from_this<gympp::Environment>
{
private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl = nullptr;

protected:
    using GazeboServer = ignition::gazebo::Server;
    std::unique_ptr<GazeboServer, std::function<void(GazeboServer*)>> m_server = nullptr;

public:
    using Environment = gympp::Environment;
    using Environment::Action;
    using Environment::Observation;
    using Environment::RenderMode;
    using Environment::Reward;
    using Environment::State;

    IgnitionGazebo() = delete;
    IgnitionGazebo(const std::string libName,
                   const std::string& pluginName,
                   const ActionSpacePtr aSpace,
                   const ObservationSpacePtr oSpace,
                   const std::string& sdfFile,
                   double updateRate,
                   uint64_t iterations = 1);
    ~IgnitionGazebo() override;

    bool render(RenderMode mode) override;
    std::optional<Observation> reset() override;
    std::optional<State> step(const Action& action) override;
    std::vector<unsigned> seed(unsigned seed = 0) override;

    // Public APIs
    EnvironmentPtr env();
    void setVerbosity(int level = 4);
    bool loadSDF(std::string& sdfFile);
};

#endif // GYMPP_GYMS_IGNITION
