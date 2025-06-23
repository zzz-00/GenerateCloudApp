#pragma once

#include <string>

#include "task.h"

class TaskManager
{
public:
    virtual ~TaskManager() = default;

    virtual void run() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;

    virtual void startTask(int id, Task *task) = 0;
    virtual void stopTask(int id) = 0;
    virtual void stopAllTask() = 0;
};