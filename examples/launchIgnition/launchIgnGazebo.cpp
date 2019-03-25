#include <cstdlib>
#include <iostream>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <ignition/gazebo/gui/TmpIface.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

// Refer to ign-gazebo/src/main.cc

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cout << "A single input argument containing the sdf file is required" << std::endl;
        return EXIT_FAILURE;
    }

    // Set verbosity
    ignition::common::Console::SetVerbosity(4);

    std::string sdfFile(argv[1]);

    ignition::gazebo::ServerConfig serverConfig;
    serverConfig.SetUpdateRate(10); // 10 Hz

    if (!serverConfig.SetSdfFile(sdfFile)) {
        std::cout << "Failed to set the SDF file " << sdfFile << std::endl;
        return EXIT_FAILURE;
    }

    // TODO: Allow running gazebo either headless or with GUI.
    //       For the time being only GUI is supported.

    // Temporary transport interface
    auto tmp = std::make_unique<ignition::gazebo::TmpIface>();

    // Initialize Qt app
    ignition::gui::Application app(argc, argv);

    // Load configuration file
    auto configPath = ignition::common::joinPaths(IGNITION_GAZEBO_GUI_CONFIG_PATH, "gui.config");

    if (!app.LoadConfig(configPath)) {
        return EXIT_FAILURE;
    }

    // Customize window
    auto win = app.findChild<ignition::gui::MainWindow*>()->QuickWindow();
    win->setProperty("title", "Gazebo");

    // Let QML files use TmpIface' functions and properties
    auto context = new QQmlContext(app.Engine()->rootContext());
    context->setContextProperty("TmpIface", tmp.get());

    // Instantiate GazeboDrawer.qml file into a component
    QQmlComponent component(app.Engine(), ":/Gazebo/GazeboDrawer.qml");
    auto gzDrawerItem = qobject_cast<QQuickItem*>(component.create(context));
    if (gzDrawerItem) {
        // C++ ownership
        QQmlEngine::setObjectOwnership(gzDrawerItem, QQmlEngine::CppOwnership);

        // Add to main window
        auto parentDrawerItem = win->findChild<QQuickItem*>("sideDrawer");
        gzDrawerItem->setParentItem(parentDrawerItem);
        gzDrawerItem->setParent(app.Engine());
    }
    else {
        std::cout << "Failed to instantiate custom drawer, drawer will be empty" << std::endl;
    }

    // Create the server
    auto server = std::make_unique<ignition::gazebo::Server>(serverConfig);

    // TODO Load Environment plugin
    // TODO

    // ========================
    // python::openai_gym::step
    // ========================

    // Run the server, and don't block.
    server->Run(/*_blocking=*/false, /*_iterations=*/0, /*_paused=*/false);

    // Run main window.
    // This blocks until the window is closed or we receive a SIGINT
    app.exec();

    return EXIT_SUCCESS;
}
