#include "gympp/gyms/Ignition.h"
#include "gympp/Log.h"

#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/gui/TmpIface.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

using IgnitionServer = ignition::gazebo::v0::Server;

template class gympp::gyms::IgnitionGazebo<size_t, double>;
// template class gympp::gyms::IgnitionGazebo<double, double>;
// template class gympp::gyms::IgnitionGazebo<int, float>;

using namespace gympp::gyms;

struct GuiData
{
    struct InputArgs
    {
        int argc = 0;
        const char* argv[0];
    } inputArgs;

    std::mutex mutex;
    std::condition_variable cv;

    std::unique_ptr<std::thread> thread = nullptr;
    std::shared_ptr<ignition::gui::Application> app = nullptr;
    std::unique_ptr<ignition::gazebo::TmpIface> transport = nullptr;

    static void initializeAndRun(GuiData& gui)
    {
        gymppDebug << "Creating the application" << std::endl << std::flush;

        if (!GuiData::initializeApplication(gui)) {
            gymppError << "Failed to initialize application" << std::endl;
            return;
        }
        else {
            gymppDebug << "Application created" << std::endl;
        }

        gymppError << "QUA" << std::endl << std::flush;
        {
            std::unique_lock lock(gui.mutex);
            gymppDebug << "Waiting that the server starts" << std::endl << std::flush;
            gui.cv.wait(lock);
        }

        // This blocks until the window is closed or we receive a SIGINT
        gymppDebug << "Executing the application" << std::endl;
        gui.app->exec();
    }

    static bool initializeApplication(GuiData& gui);
};

bool GuiData::initializeApplication(GuiData& gui)
{
    // Lock the mutex
    std::lock_guard lock(gui.mutex);

    // Temporary transport interface
    gui.transport = std::make_unique<ignition::gazebo::TmpIface>();

    // Initialize Qt app
    gui.app = std::make_shared<ignition::gui::Application>(gui.inputArgs.argc,
                                                           const_cast<char**>(gui.inputArgs.argv));

    // Load configuration file
    auto configPath = ignition::common::joinPaths(IGNITION_GAZEBO_GUI_CONFIG_PATH, "gui.config");

    if (!gui.app->LoadConfig(configPath)) {
        gymppError << "Failed to load Gazebo GUI configuration" << std::endl;
        return false;
    }

    // Customize window
    auto win = gui.app->findChild<ignition::gui::MainWindow*>()->QuickWindow();
    win->setProperty("title", "Gazebo");

    // Let QML files use TmpIface' functions and properties
    auto context = new QQmlContext(gui.app->Engine()->rootContext());
    context->setContextProperty("TmpIface", gui.transport.get());

    // Instantiate GazeboDrawer.qml file into a component
    QQmlComponent component(gui.app->Engine(), ":/Gazebo/GazeboDrawer.qml");
    auto gzDrawerItem = qobject_cast<QQuickItem*>(component.create(context));
    if (gzDrawerItem) {
        // C++ ownership
        QQmlEngine::setObjectOwnership(gzDrawerItem, QQmlEngine::CppOwnership);

        // Add to main window
        auto parentDrawerItem = win->findChild<QQuickItem*>("sideDrawer");
        gzDrawerItem->setParentItem(parentDrawerItem);
        gzDrawerItem->setParent(gui.app->Engine());
    }
    else {
        gymppError << "Failed to instantiate custom drawer, drawer will be empty" << std::endl;
    }

    return true;
}

template <typename AType, typename OType>
class IgnitionGazebo<AType, OType>::Impl
{
public:
    bool firstRun = true;

    std::string sdfFile;

    uint64_t numOfIterations = 0;
    ignition::gazebo::ServerConfig serverConfig;

    GuiData gui;
    bool initializeApplication();
};

template <typename AType, typename OType>
IgnitionGazebo<AType, OType>::IgnitionGazebo(const ActionSpacePtr aSpace,
                                             const ObservationSpacePtr oSpace,
                                             const std::string& sdfFile,
                                             double updateRate,
                                             uint64_t iterations)
    : Environment(aSpace, oSpace)
    , pImpl{new IgnitionGazebo::Impl}
{
    setVerbosity(4);
    pImpl->sdfFile = sdfFile;
    pImpl->numOfIterations = iterations;
    pImpl->serverConfig.SetUpdateRate(updateRate);
}

template <typename AType, typename OType>
gympp::gyms::IgnitionGazebo<AType, OType>::~IgnitionGazebo() = default;

template <typename AType, typename OType>
std::optional<typename IgnitionGazebo<AType, OType>::State>
IgnitionGazebo<AType, OType>::step(const Action& action)
{
    if (pImpl->firstRun) {
        pImpl->firstRun = false;

        // Load the sdf file from the filesystem
        // TODO: get the absolute path with Ignition::Filesystem?
        if (!loadSDF(pImpl->sdfFile)) {
            gymppError << "Failed to load the SDF";
            return {};
        }

        // Create the server
        gymppDebug << "Creating the server" << std::endl << std::flush;
        std::unique_lock lock(pImpl->gui.mutex);
        m_server = std::make_unique<IgnitionServer>(pImpl->serverConfig);
    }

    {
        std::unique_lock lock(pImpl->gui.mutex);
        if (pImpl->gui.thread && pImpl->gui.app) {
            // TODO: improve
            lock.unlock();

            // Run main window on a separate thread
            pImpl->gui.cv.notify_all();
        }
    }

    // Set the action to the environment
    if (!(this->action_space->contains(action) && action.get<AType>()
          && this->setAction(*action.get<AType>()))) {
        gymppError << "Failed to set the action" << std::endl;
        return {};
    }

    // Run the server synchronously
    if (!m_server->Run(/*_blocking=*/true, pImpl->numOfIterations, /*_paused=*/false)) {
        gymppError << "The server couldn't execute the step" << std::endl;
        return {};
    }

    // Get the observation from the environment
    std::optional<typename TypedEnvironmentBehavior::Observation> observation = getObservation();

    if (!(observation && this->observation_space->contains(Observation(observation.value())))) {
        gymppError << "Failed to get the observation" << std::endl;
        return {};
    }

    // Get the reward from the environment
    std::optional<Reward> reward = computeReward();

    if (!(reward && this->reward_range.contains(reward.value()))) {
        gymppError << "Failed to compute reward" << std::endl;
        return {};
    }

    State state = {isDone(), {}, reward.value(), GenericBuffer{observation.value()}};
    return state;
}

template <typename AType, typename OType>
void IgnitionGazebo<AType, OType>::setVerbosity(int level)
{
    ignition::common::Console::SetVerbosity(level);
}

template <typename AType, typename OType>
bool IgnitionGazebo<AType, OType>::loadSDF(std::string& sdfFile)
{
    if (sdfFile.empty()) {
        gymppError << "Passed SDF file is an empty string" << std::endl;
        return false;
    }

    if (!pImpl->serverConfig.SetSdfFile(sdfFile)) {
        std::cout << "Failed to set the SDF file " << sdfFile << std::endl;
        return false;
    }

    return true;
}

template <typename AType, typename OType>
typename IgnitionGazebo<AType, OType>::Environment* IgnitionGazebo<AType, OType>::env()
{
    // TODO return a share pointer to itself. There is a pattern for sharing the memory
    // by inheriting from a shared ptr particular class
    //    return static_cast<IgnitionGazebo<AType, OType>>(this);
    return static_cast<Environment*>(this);
}

template <typename AType, typename OType>
std::optional<typename IgnitionGazebo<AType, OType>::Observation>
IgnitionGazebo<AType, OType>::reset()
{
    return {};
}

template <typename AType, typename OType>
bool IgnitionGazebo<AType, OType>::render(IgnitionGazebo::RenderMode mode)
{
    if (mode == IgnitionGazebo::RenderMode::HUMAN) {
        pImpl->gui.thread =
            //            std::make_unique<std::thread>(std::bind(&GuiData::initializeAndRun,
            //            pImpl->gui));
            //            std::make_unique<std::thread>(&GuiData::initializeAndRun, pImpl->gui);
            std::make_unique<std::thread>(GuiData::initializeAndRun, std::ref(pImpl->gui));
        return true;
    }

    return false;
}
