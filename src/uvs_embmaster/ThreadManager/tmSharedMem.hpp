
/**
 * @file tmSharedMem.hpp
 * @brief Header file for the tmSharedMem class template.
 *
 * This file contains the definition of the tmSharedMem class template, which provides
 * a thread-safe mechanism for shared memory access using a timed mutex.
 */

#ifndef TMSHAREDMEM_HPP
#define TMSHAREDMEM_HPP

#include <iostream>
#include <string>

#include <future>
#include <thread>
#include <chrono>
#include <functional>
#include <mutex>
#include <condition_variable>

/**
* @class tmSharedMem
* @brief A template class for thread-safe shared memory access.
*
* The tmSharedMem class template provides a mechanism for safely accessing shared
* memory in a multi-threaded environment using a timed mutex.
*
* @tparam T The type of the data to be shared.
*/
template <typename T>
class tmSharedMem
{
public:
    /**
    * @brief Default constructor.
    *
    * Initializes the shared memory with a default-constructed object of type T.
    */
    tmSharedMem() : data(T()) {}
    /**
    * @brief Parameterized constructor.
    *
    * Initializes the shared memory with a given object of type T.
    *
    * @param data The initial data to be stored in the shared memory.
    */
    tmSharedMem(const T &data) : data(data) {}

    /**
    * @brief Deleted copy constructor.
    *
    * Copying of tmSharedMem objects is not allowed.
    */
    tmSharedMem(const tmSharedMem &) = delete;
    /**
    * @brief Deleted move constructor.
    *
    * Moving of tmSharedMem objects is not allowed.
    */
    tmSharedMem(tmSharedMem &&) = delete;
    /**
    * @brief Deleted copy assignment operator.
    *
    * Copy assignment of tmSharedMem objects is not allowed.
    *
    * @return A reference to the current object.
    */
    tmSharedMem &operator=(const tmSharedMem &) = delete;

    /**
    * @brief Attempts to acquire the lock on the shared memory.
    *
    * Tries to lock the shared memory for a specified duration.
    *
    * @param timeout The maximum duration to wait for the lock. Default is 0 milliseconds.
    * @return True if the lock was acquired, false otherwise.
    */
    bool acquire(std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
    {
        return mtx.try_lock_for(timeout);
    }
    /**
    * @brief Accesses the shared data.
    *
    * Provides access to the shared data. The caller must ensure that the lock
    * has been acquired before calling this method.
    *
    * @return A reference to the shared data.
    */
    T& get()
    {
        return data;
    }
    /**
    * @brief Releases the lock on the shared memory.
    *
    * Unlocks the shared memory. The caller must ensure that the lock
    * has been acquired before calling this method.
    */
    void release()
    {
        mtx.unlock();
    }
    
private:
    T data;
    std::timed_mutex mtx;
};

#endif // TMSHAREDMEM_HPP
