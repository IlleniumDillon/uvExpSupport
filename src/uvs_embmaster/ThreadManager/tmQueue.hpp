#ifndef TMQUEUE_HPP
#define TMQUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>

template<typename T, size_t len>
class tmQueue {
public:
    tmQueue() = default;
    ~tmQueue() = default;

    bool get(T &data, std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
    {
        std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
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
        return false;
    }

    bool put(const T &data, std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
    {
        std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
        do
        {
            if(mtx.try_lock_for(std::chrono::milliseconds(0)))
            {
                if (q.size() >= len)
                {
                    q.pop();
                }
                q.push(data);
                mtx.unlock();
                return true;
            }
        }while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start) < timeout);
        return false;
    }

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