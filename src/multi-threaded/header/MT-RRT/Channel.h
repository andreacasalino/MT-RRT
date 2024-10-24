/**
 * Author:    Andrea Casalino
 * Created:   01.11.2024
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <vector>
#include <atomic>

namespace mt_rrt {
// single producer single consumer queue
template<typename T>
class Channel {
public:
    Channel(std::size_t capacity)
    {
        buffer_.resize(capacity);
    }

    template<typename ... ARGS>
    bool push(ARGS&& ... args) {
        std::size_t producer_cursor = producer_cursor_.load(std::memory_order::memory_order_acquire);
        std::size_t size = producer_cursor - consumer_cursor_.load(std::memory_order::memory_order_acquire);
        if(buffer_.size() <= size) {
            return false;
        }
        std::size_t index = producer_cursor % buffer_.size();
        buffer_[index] = T{std::forward<ARGS>(args)...};
        producer_cursor_.store(producer_cursor + 1, std::memory_order::memory_order_release);
        return true;
    }

    bool poll(T& recipient) {
        std::size_t consumer_cursor = consumer_cursor_.load(std::memory_order::memory_order_acquire);
        if(consumer_cursor == producer_cursor_.load(std::memory_order::memory_order_acquire)) {
            return false;
        }
        std::size_t index = consumer_cursor % buffer_.size();
        recipient = std::move(buffer_[index]);
        consumer_cursor_.store(consumer_cursor + 1, std::memory_order::memory_order_release);
        return true;
    }

private:
    std::atomic<std::size_t> producer_cursor_{0};
    std::atomic<std::size_t> consumer_cursor_{0};
    std::vector<T> buffer_;
};
}
