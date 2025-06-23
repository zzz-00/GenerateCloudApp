#pragma once

#include "task_manager.h"

#include <mutex>
#include <unordered_map>

#include "logger.h"

class TaskManagerImpl : public TaskManager
{
public:
    static TaskManagerImpl &instance();

    virtual void run() override;
    virtual void start() override;
    virtual void stop() override;

    virtual void startTask(int id, Task *task) override;
    virtual void stopTask(int id) override;
    virtual void stopAllTask() override;

private:
    TaskManagerImpl() {};
    TaskManagerImpl(const TaskManagerImpl &) = delete;
    TaskManagerImpl &operator=(const TaskManagerImpl &) = delete;

    std::thread worker_;
    std::atomic<bool> running_{false};
    std::mutex mutex_;

    std::unordered_map<int, Task *> tasks_;
};