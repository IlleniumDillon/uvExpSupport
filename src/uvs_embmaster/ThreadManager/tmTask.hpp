
/**
 * @file tmTask.hpp
 * @brief Header file for the tmTask class, which provides a mechanism for managing tasks with the ability to stop them gracefully.
 * 
 * This file defines the tmTask class, which includes functionality for running tasks, stopping them, and checking if a stop has been requested.
 */


#ifndef TMTASK_HPP
#define TMTASK_HPP

#include <iostream>
#include <string>

#include <future>
#include <thread>
#include <chrono>
#include <functional>

/**
 * @class tmTask
 * @brief A class to manage tasks with the ability to stop them gracefully.
 * 
 * The tmTask class provides a mechanism to run tasks, stop them, and check if a stop has been requested.
 * It uses std::promise and std::future to signal and check for stop requests.
 * 
 * @note Copy constructor and copy assignment operator are deleted to prevent copying.
 * 
 * @param taskName The name of the task.
 */
class tmTask
{
public:
    /**
     * @brief Constructor for tmTask.
     * 
     * Constructs a tmTask with the specified task name.
     */
    tmTask(std::string taskName) 
        : taskName(taskName), futureObj(exitSignal.get_future())
        {}
    /**
     * @brief Default destructor for tmTask.
     * 
     * Destructs the tmTask.
     */
    ~tmTask(){}

    tmTask(const tmTask&) = delete;
    tmTask& operator=(const tmTask&) = delete;
    /**
     * @brief Main function of the task.
     * 
     * This function should be overridden by derived classes to define the task's behavior.
     */
    virtual void run(const bool &){};
    // void operator()() { run(); }
    /**
     * @brief Stop the task.
     */
    void stop() { exitSignal.set_value(); }
    /**
     * @brief Check if a stop has been requested.
     */
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