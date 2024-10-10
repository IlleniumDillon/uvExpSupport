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

template <typename T>
class tmSharedMem
{
public:
    tmSharedMem() : data(T()) {}
    tmSharedMem(const T &data) : data(data) {}

    tmSharedMem(const tmSharedMem &) = delete;
    tmSharedMem(tmSharedMem &&) = delete;
    tmSharedMem &operator=(const tmSharedMem &) = delete;

    bool acquire(std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
    {
        return mtx.try_lock_for(timeout);
    }
    T& get()
    {
        return data;
    }
    void release()
    {
        mtx.unlock();
    }
    
private:
    T data;
    std::timed_mutex mtx;
};

#endif // TMSHAREDMEM_HPP