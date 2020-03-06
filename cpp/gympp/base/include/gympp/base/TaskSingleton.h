/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef GYMPP_BASE_TASKSINGLETON_H
#define GYMPP_BASE_TASKSINGLETON_H

#include <memory>
#include <string>
#include <unordered_map>

namespace gympp {
    namespace base {
        class Task;
        class TaskSingleton;
    } // namespace base
} // namespace gympp

class gympp::base::TaskSingleton
{
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

protected:
public:
    using TaskName = std::string;

    TaskSingleton();
    ~TaskSingleton();
    TaskSingleton(TaskSingleton&) = delete;
    void operator=(const TaskSingleton&) = delete;

    static TaskSingleton& get();

    gympp::base::Task* getTask(const TaskName& taskName);

    bool storeTask(const TaskName& taskName, gympp::base::Task* task);
    bool removeTask(const TaskName& taskName);
};

#endif // GYMPP_BASE_TASKSINGLETON_H
