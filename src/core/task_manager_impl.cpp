#include "task_manager_impl.h"

TaskManagerImpl &TaskManagerImpl::instance()
{
    static TaskManagerImpl instance;
    return instance;
}

void TaskManagerImpl::run()
{
    while (running_.load())
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);

            for (auto it = tasks_.begin(); it != tasks_.end();)
            {
                auto &[id, task] = *it;
                if (task)
                {
                    if (!task->isFinished())
                    {
                        if (!task->isRunning())
                        {
                            task->start();
                            Logger::instance().log(LogLevel::Info, "Task " + std::to_string(id) + " started.");
                        }
                        ++it;
                    }
                    else
                    {
                        Logger::instance().log(LogLevel::Info, "Task " + std::to_string(id) + " finished and removed.");
                        it = tasks_.erase(it);
                    }
                }
                else
                {
                    ++it;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void TaskManagerImpl::start()
{
    if (!running_.load())
    {
        running_.store(true);
        worker_ = std::thread(&TaskManagerImpl::run, this);
    }
}

void TaskManagerImpl::stop()
{
    this->stopAllTask();

    running_.store(false);
    if (worker_.joinable())
    {
        worker_.join();
    }
}

void TaskManagerImpl::startTask(int id, Task *task)
{
    Task *oldTask = nullptr;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = tasks_.find(id);
        if (it != tasks_.end())
        {
            Logger::instance().log(LogLevel::Warn, "Task " + std::to_string(id) + " already exists. Stopping old task.");
            oldTask = it->second;
            tasks_.erase(it);
        }
    }

    if (oldTask)
    {
        oldTask->stop();
        delete oldTask;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        tasks_[id] = task;
    }

    Logger::instance().log(LogLevel::Info, "Task " + std::to_string(id) + " is starting...");
    task->start();
}

void TaskManagerImpl::stopTask(int id)
{
    Task *taskToStop = nullptr;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = tasks_.find(id);
        if (it != tasks_.end())
        {
            taskToStop = it->second;
            tasks_.erase(it);
        }
    }

    if (taskToStop)
    {
        Logger::instance().log(LogLevel::Info, "Stopping Task " + std::to_string(id));
        taskToStop->stop();
    }
}

void TaskManagerImpl::stopAllTask()
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto &[id, task] : tasks_)
    {
        if (task)
        {
            task->stop();
            delete task;
        }
    }

    tasks_.clear();
    std::string msg = "All tasks stopped.";
    Logger::instance().log(LogLevel::Info, msg);
}