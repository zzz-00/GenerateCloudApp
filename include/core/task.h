#pragma once

#include <atomic>
#include <thread>

class Task
{
public:
    explicit Task(int id) : id_(id), running_(false), finished_(false) {}
    virtual ~Task()
    {
        if (task_thread_.joinable())
        {
            task_thread_.join();
        }
    }

    void start()
    {
        if (running_.load())
        {
            return;
        }

        if (task_thread_.joinable())
        {
            task_thread_.join();
        }

        running_.store(true);
        finished_.store(false);

        task_thread_ = std::thread([this]()
                                   {
                                       this->run();
                                       finished_.store(true);
                                       running_.store(false); });
    }

    void stop()
    {
        if (!running_.load())
        {
            return;
        }
        running_.store(false);

        if (task_thread_.joinable())
        {
            task_thread_.join();
        }
        finished_.store(false);
    }

    bool isRunning() const { return running_.load(); }
    bool isFinished() const { return finished_.load(); }
    int getId() const { return id_; }

protected:
    virtual void run() = 0;

    int id_;
    std::atomic<bool> running_;
    std::atomic<bool> finished_;
    std::thread task_thread_;
};
