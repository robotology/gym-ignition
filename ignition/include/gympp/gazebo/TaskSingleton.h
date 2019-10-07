/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_ROBOT_TASKSINGLETON_H
#define GYMPP_ROBOT_TASKSINGLETON_H

#include <ignition/common/SingletonT.hh>

#include <string>
#include <unordered_map>

namespace gympp {
    namespace gazebo {
        class Task;
        class TaskSingleton;
    } // namespace gazebo
} // namespace gympp

class gympp::gazebo::TaskSingleton : public ignition::common::SingletonT<TaskSingleton>
{
private:
    static std::unordered_map<std::string, Task*> m_tasks;

protected:
public:
    using TaskName = std::string;

    TaskSingleton() = default;
    ~TaskSingleton() override = default;

    gympp::gazebo::Task* get(const TaskName& taskName);

    bool storeTask(const TaskName& taskName, gympp::gazebo::Task* task);
    bool removeTask(const TaskName& taskName);
};

#endif // GYMPP_ROBOT_TASKSINGLETON_H
