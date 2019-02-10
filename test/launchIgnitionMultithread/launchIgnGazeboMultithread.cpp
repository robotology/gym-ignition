#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
//#include <ignition/gazebo/gui/TmpIface.hh>
//#include <ignition/gui/Application.hh>
//#include <ignition/gui/MainWindow.hh>

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
    serverConfig.SetUpdateRate(100); // 10 Hz

    if (!serverConfig.SetSdfFile(sdfFile)) {
        std::cout << "Failed to set the SDF file " << sdfFile << std::endl;
        return EXIT_FAILURE;
    }

    // Create the server
    auto server = std::make_unique<ignition::gazebo::Server>(serverConfig);

    // Run the GUI on another process
    pid_t guiPid;
    guiPid = fork();
    if (guiPid == 0) {
        // remove client from foreground process group
        setpgid(guiPid, 0);

        // Spin up GUI process and block here
        char** argvGui = new char*[2];
        argvGui[0] = const_cast<char*>("ign-gazebo-gui");
        argvGui[1] = nullptr;
        execvp("ign-gazebo-gui", argvGui);
        ignerr << "Failed to execute GUI";
    }

    // TODO Load Environment plugin
    // TODO

    // ========================
    // python::openai_gym::step
    // ========================

    // Run the server, and don't block.
    //    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    server->Run(/*_blocking=*/true, /*_iterations=*/100, /*_paused=*/true);
    //    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Run main window.
    // This blocks until the window is closed or we receive a SIGINT
    // app.exec();

    return EXIT_SUCCESS;
}
