#ifndef TMTASK_HPP
#define TMTASK_HPP

#include <iostream>
#include <string>

#include <future>
#include <thread>
#include <chrono>
#include <functional>

class tmTask
{
    friend class ThreadManager;
public:
    tmTask(std::string taskName) 
        : taskName(taskName), futureObj(exitSignal.get_future())
        {}
    ~tmTask(){}

    tmTask(const tmTask&) = delete;
    tmTask& operator=(const tmTask&) = delete;

    virtual void run(const bool &){};
    // void operator()() { run(); }

    void stop() { exitSignal.set_value(); }

    bool stopRequested()
    {
        return !(futureObj.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout);
    }

private:
    std::string taskName;

    std::promise<void> exitSignal;
    std::future<void> futureObj;
};

#endif // TMTASK_HPP