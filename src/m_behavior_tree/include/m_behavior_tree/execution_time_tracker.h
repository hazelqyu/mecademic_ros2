#ifndef EXECUTION_TIME_TRACKER_H
#define EXECUTION_TIME_TRACKER_H

#include <chrono>
#include <mutex>

class ExecutionTimeTracker {
public:
    static ExecutionTimeTracker& getInstance() {
        static ExecutionTimeTracker instance;
        return instance;
    }

    // Update the last execution time
    void updateLastExecutionTime() {
        std::lock_guard<std::mutex> lock(mutex_);
        last_execution_time_ = std::chrono::steady_clock::now();
    }

    // Get the last execution time
    std::chrono::steady_clock::time_point getLastExecutionTime() {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_execution_time_;
    }

private:
    ExecutionTimeTracker() = default;
    std::mutex mutex_;  // To ensure thread safety
    std::chrono::steady_clock::time_point last_execution_time_{std::chrono::steady_clock::time_point::min()};
};

#endif  // EXECUTION_TIME_TRACKER_H
