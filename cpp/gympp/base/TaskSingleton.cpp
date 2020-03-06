/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "gympp/base/TaskSingleton.h"
#include "gympp/base/Log.h"
#include "gympp/base/Task.h"

#include <cassert>
#include <ostream>

using namespace gympp::base;

class TaskSingleton::Impl
{
public:
    std::unordered_map<TaskName, Task*> tasks;
};

TaskSingleton::TaskSingleton()
    : pImpl{new Impl()}
{}

TaskSingleton::~TaskSingleton()= default;

gympp::base::TaskSingleton& TaskSingleton::get()
{
    static TaskSingleton instance;
    return instance;
}

Task* TaskSingleton::getTask(const TaskName& taskName)
{
    if (pImpl->tasks.find(taskName) == pImpl->tasks.end()) {
        gymppError << "Failed to find Task '" << taskName << "'" << std::endl;
        return nullptr;
    }

    assert(pImpl->tasks.at(taskName));
    return pImpl->tasks.at(taskName);
}

bool TaskSingleton::storeTask(const TaskName& taskName, Task* task)
{
    if (!task || taskName.empty()) {
        gymppError << "Trying to store an invalid Task interface" << std::endl;
        return false;
    }

    if (pImpl->tasks.find(taskName) != pImpl->tasks.end()) {
        gymppError << "Task '" << taskName << "' have been already registered" << std::endl;
    }

    gymppDebug << "Storing Task '" << taskName << "'" << std::endl;
    pImpl->tasks[taskName] = task;

    return true;
}

bool TaskSingleton::removeTask(const TaskName& taskName)
{
    if (taskName.empty()) {
        gymppError << "The label of the tasks to delete is empty" << std::endl;
        return false;
    }

    if (pImpl->tasks.find(taskName) == pImpl->tasks.end()) {
        gymppError << "The task '" << taskName << "' have never been stored" << std::endl;
        return false;
    }

    gymppDebug << "Deleting task '" << taskName << "'" << std::endl;
    pImpl->tasks.erase(taskName);
    return true;
}
