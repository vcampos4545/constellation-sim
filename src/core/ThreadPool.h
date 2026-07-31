#pragma once

#include <algorithm>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <atomic>
#include <stdexcept>

// Fixed-size thread pool for Monte Carlo parallelism.
// Threads are created once at construction; tasks are submitted as std::function<void()>.
// The destructor drains the queue and joins all threads cleanly.
class ThreadPool {
public:
    explicit ThreadPool(unsigned int num_threads = 0) {
        const unsigned int n = (num_threads == 0)
            ? std::max(1u, std::thread::hardware_concurrency())
            : num_threads;

        workers_.reserve(n);
        for (unsigned int i = 0; i < n; ++i) {
            workers_.emplace_back([this] { workerLoop(); });
        }
    }

    ~ThreadPool() {
        {
            std::unique_lock lock(mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& t : workers_) t.join();
    }

    ThreadPool(const ThreadPool&)            = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // Submit a callable and return a future for its result.
    template<typename F, typename... Args>
    auto submit(F&& f, Args&&... args) -> std::future<std::invoke_result_t<F, Args...>> {
        using Ret = std::invoke_result_t<F, Args...>;
        auto task = std::make_shared<std::packaged_task<Ret()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...));
        std::future<Ret> fut = task->get_future();
        {
            std::unique_lock lock(mutex_);
            if (stop_) throw std::runtime_error("ThreadPool is stopped");
            queue_.emplace([task] { (*task)(); });
        }
        cv_.notify_one();
        return fut;
    }

    unsigned int size() const { return static_cast<unsigned int>(workers_.size()); }

private:
    void workerLoop() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock lock(mutex_);
                cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
                if (stop_ && queue_.empty()) return;
                task = std::move(queue_.front());
                queue_.pop();
            }
            task();
        }
    }

    std::vector<std::thread>        workers_;
    std::queue<std::function<void()>> queue_;
    std::mutex                      mutex_;
    std::condition_variable         cv_;
    bool                            stop_{false};
};
