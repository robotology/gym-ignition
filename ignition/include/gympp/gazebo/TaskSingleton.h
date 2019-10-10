/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_ROBOT_TASKSINGLETON_H
#define GYMPP_ROBOT_TASKSINGLETON_H

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace gympp {
    namespace gazebo {
        class Task;
        class TaskSingleton;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::TaskSingleton
{
private:
    class Impl;
    std::unique_ptr<Impl, std::function<void(Impl*)>> pImpl;

protected:
public:
    using TaskName = std::string;

    TaskSingleton();
    ~TaskSingleton() = default;
    TaskSingleton(TaskSingleton&) = delete;
    void operator=(const TaskSingleton&) = delete;

    static TaskSingleton& get();

    gympp::gazebo::Task* getTask(const TaskName& taskName);

    bool storeTask(const TaskName& taskName, gympp::gazebo::Task* task);
    bool removeTask(const TaskName& taskName);
};

#endif // GYMPP_ROBOT_TASKSINGLETON_H
