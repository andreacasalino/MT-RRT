/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <atomic>

namespace mt_rrt {

class SpinLock {
public:
    SpinLock() = default;

    struct Guard {
        Guard(SpinLock& lock)
        : lock_{lock} 
        {
            lock_.lock();
        }

        ~Guard() {
            lock_.unlock();
        }

    private:
        SpinLock& lock_;
    };

private:
    void lock() {
        do {
            bool expected{true};
            if(flag.compare_exchange_strong(expected, false, std::memory_order::memory_order_acquire)) {
                break;
            }
        } while (true);
    }
    
    void unlock() {
        flag.store(true, std::memory_order::memory_order_release);
    }

    std::atomic_bool flag{true};
};

using SpinLockGuard = SpinLock::Guard;
}
