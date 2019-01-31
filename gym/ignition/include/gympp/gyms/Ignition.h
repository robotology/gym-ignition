#ifndef GYMPP_GYMS_IGNITION
#define GYMPP_GYMS_IGNITION

#include "gympp/Gympp.h"

// TODO: fix forward decl
#include <ignition/gazebo/Server.hh>

#include <memory>

namespace gympp {
    namespace gyms {
        template <typename AType, typename OType>
        class IgnitionGazebo;
        //        template <typename AType, typename OType>
        class EnvironmentBehavior;
    } // namespace gyms
} // namespace gympp

// TODO
// namespace ignition {
//    namespace gazebo {
//        inline namespace v0 {
//            class Server;
//        }
//    } // namespace gazebo
//} // namespace ignition

// template <typename ActionDataType, typename ObservationDataType>
class gympp::gyms::EnvironmentBehavior
{
public:
    //    template <typename Type>
    //    using Buffer = std::valarray<Type>;

    //    using Action = Buffer<ActionDataType>;
    //    using Observation = Buffer<ObservationDataType>;
    using Action = gympp::Environment::Action;
    using Observation = gympp::Environment::Observation;
    using Reward = gympp::Environment::Reward;

    virtual ~EnvironmentBehavior() = default;

    virtual bool isDone() = 0;
    virtual bool setAction(const Action& action) = 0;
    virtual std::optional<Reward> computeReward() = 0;
    virtual std::optional<Observation> getObservation() = 0;
};

template <typename AType, typename OType>
class gympp::gyms::IgnitionGazebo : public gympp::Environment
//    , gympp::gyms::EnvironmentBehavior
//    , gympp::gyms::EnvironmentBehavior<AType, OType>
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl = nullptr;

protected:
    //    std::unique_ptr<ignition::gazebo::v0::Server> m_server = nullptr;
    std::unique_ptr<ignition::gazebo::Server> m_server = nullptr;

public:
    using Environment = gympp::Environment;
    //    using State = typename Environment::State;
    using Environment::Action;
    using Environment::State;
    //    using Action = typename Environment::Action;
    //    using Reward = typename Environment::Reward;
    using Environment::Reward;
    //    using Observation = typename Environment::Observation;
    using Environment::Observation;
    using Environment::RenderMode;
    //    using RenderMode = typename Environment::RenderMode;
    //    using ActionSpace = typename Environment::ActionSpace;
    //    using ObservationSpace = typename Environment::ObservationSpace;
    //    using ActionSpacePtr = typename Environment::ActionSpacePtr;
    //    using ObservationSpacePtr = typename Environment::ObservationSpacePtr;

    //    using TypedEnvironmentBehavior = EnvironmentBehavior<AType, OType>;

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

    // Public APIs
    Environment* env();
    void setVerbosity(int level = 4);
    bool loadSDF(std::string& sdfFile);
};

// Instantiate the template for canonical problems
extern template class gympp::gyms::IgnitionGazebo<size_t, double>;

#endif // GYMPP_GYMS_IGNITION
