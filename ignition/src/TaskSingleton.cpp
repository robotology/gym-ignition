/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/gazebo/TaskSingleton.h"
#include "gympp/Log.h"
#include "gympp/gazebo/Task.h"

#include <cassert>
#include <ostream>

using namespace gympp::gazebo;

std::unordered_map<TaskSingleton::TaskName, Task*> gympp::gazebo::TaskSingleton::m_tasks = {};

Task* TaskSingleton::get(const TaskName& taskName)
{
    if (m_tasks.find(taskName) == m_tasks.end()) {
        gymppError << "Failed to find Task '" << taskName << "'" << std::endl;
        return nullptr;
    }

    assert(m_tasks.at(taskName));
    return m_tasks.at(taskName);
}

bool TaskSingleton::storeTask(const TaskName& taskName, Task* task)
{
    if (!task || taskName.empty()) {
        gymppError << "Trying to store an invalid Task interface" << std::endl;
        return false;
    }

    if (m_tasks.find(taskName) != m_tasks.end()) {
        gymppError << "Task '" << taskName << "' have been already registered" << std::endl;
    }

    gymppDebug << "Storing Task '" << taskName << "'" << std::endl;
    m_tasks[taskName] = task;

    return true;
}

bool TaskSingleton::removeTask(const TaskName& taskName)
{
    if (taskName.empty()) {
        gymppError << "The label of the tasks to delete is empty" << std::endl;
        return false;
    }

    if (m_tasks.find(taskName) == m_tasks.end()) {
        gymppError << "The task '" << taskName << "' have never been stored" << std::endl;
        return false;
    }

    gymppDebug << "Deleting task '" << taskName << "'" << std::endl;
    m_tasks.erase(taskName);
    return true;
}
