
/**
* @file tmQueue.hpp
* @brief A thread-safe queue implementation with a fixed maximum length.
* 
* This header file defines the tmQueue class template, which provides a thread-safe
* queue with a fixed maximum length. The queue supports timed get and put operations.
*/
#ifndef TMQUEUE_HPP
#define TMQUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>

/**
* @class tmQueue
* @brief A thread-safe queue with a fixed maximum length.
* 
* The tmQueue class provides a thread-safe queue with a fixed maximum length. It supports
* timed get and put operations, allowing elements to be added and removed from the queue
* with a specified timeout.
* 
* @tparam T The type of elements stored in the queue.
* @tparam len The maximum length of the queue.
*/
template<typename T, size_t len>
class tmQueue {
public:
    /**
    * @brief Default constructor.
    * 
    * Constructs an empty tmQueue.
    */
    tmQueue() = default;
    /**
    * @brief Default destructor.
    * 
    * Destructs the tmQueue.
    */
    ~tmQueue() = default;
    /**
    * @brief Retrieves an element from the queue.
    * 
    * Attempts to retrieve an element from the queue within the specified timeout period.
    * 
    * @param data Reference to the variable where the retrieved element will be stored.
    * @param timeout The maximum time to wait for an element to become available.
    * @return true if an element was successfully retrieved, false otherwise.
    */
    bool get(T &data, std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
    {
        std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
        // Try to lock the mutex and retrieve the front element from the queue
        do
        {
            if(mtx.try_lock_for(std::chrono::milliseconds(0)))
            {
                if(!q.empty())
                {
                    data = q.front();
                    q.pop();
                    mtx.unlock();
                    return true;
                }
                mtx.unlock();
            }
        }while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start) < timeout);

        // No data available
        return false;
    }
    /**
    * @brief Adds an element to the queue.
    * 
    * Attempts to add an element to the queue within the specified timeout period. If the queue
    * is full, the oldest element will be removed to make space for the new element.
    * 
    * @param data The element to be added to the queue.
    * @param timeout The maximum time to wait for space to become available in the queue.
    * @return true if the element was successfully added, false otherwise.
    */
    bool put(const T &data, std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
    {
        std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
        // Try to lock the mutex and add the element to the queue
        do
        {
            if(mtx.try_lock_for(std::chrono::milliseconds(0)))
            {
                // If the queue is full, remove the oldest element
                if (q.size() >= len)
                {
                    q.pop();
                }
                q.push(data);
                mtx.unlock();
                return true;
            }
        }while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start) < timeout);

        // No space available
        return false;
    }
    /**
    * @brief Returns the number of elements in the queue.
    * 
    * Retrieves the current number of elements in the queue.
    * 
    * @return The number of elements in the queue.
    */
    size_t size()
    {
        std::lock_guard<std::timed_mutex> lock(mtx);
        return q.size();
    }

private:
    std::queue<T> q;
    std::timed_mutex mtx;
};

#endif // TMQUEUE_HPP
