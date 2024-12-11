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
    void updateLastExecutionTime(const std::string& node_name) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_execution_times_[node_name] = std::chrono::steady_clock::now();
        global_last_execution_time_ = std::chrono::steady_clock::now();
    }

    // Get the last execution time
    std::chrono::steady_clock::time_point getLastExecutionTime(const std::string& node_name) {
        std::lock_guard<std::mutex> lock(mutex_);
        // return last_execution_time_;
        if (last_execution_times_.find(node_name) != last_execution_times_.end()) {
            return last_execution_times_[node_name];
        }
        return std::chrono::steady_clock::time_point::min();// Return epoch if not set
    }
    // Get the global last execution time
    std::chrono::steady_clock::time_point getGlobalLastExecutionTime() {
        std::lock_guard<std::mutex> lock(mutex_);
        return global_last_execution_time_;
    }

private:
    ExecutionTimeTracker() = default;
    std::mutex mutex_;  // To ensure thread safety
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_execution_times_;
    std::chrono::steady_clock::time_point global_last_execution_time_{std::chrono::steady_clock::time_point::min()};
};

#endif  // EXECUTION_TIME_TRACKER_H
