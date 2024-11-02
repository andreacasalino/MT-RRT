/**
 * Author:    Andrea Casalino
 * Created:   01.11.2024
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <vector>

namespace mt_rrt {
// single producer single consumer queue
template <typename T> class Channel {
public:
  Channel(std::size_t capacity) { buffer_.resize(capacity); }

  template <typename... ARGS> bool push(ARGS &&...args) {
    std::size_t producer_cursor =
        producer_cursor_.load(std::memory_order::memory_order_acquire);
    std::size_t size =
        producer_cursor -
        consumer_cursor_.load(std::memory_order::memory_order_acquire);
    if (buffer_.size() <= size) {
      return false;
    }
    std::size_t index = producer_cursor % buffer_.size();
    buffer_[index] = T{std::forward<ARGS>(args)...};
    producer_cursor_.store(producer_cursor + 1,
                           std::memory_order::memory_order_release);
    return true;
  }

  bool poll(T &recipient) {
    std::size_t consumer_cursor =
        consumer_cursor_.load(std::memory_order::memory_order_acquire);
    if (consumer_cursor ==
        producer_cursor_.load(std::memory_order::memory_order_acquire)) {
      return false;
    }
    std::size_t index = consumer_cursor % buffer_.size();
    recipient = std::move(buffer_[index]);
    consumer_cursor_.store(consumer_cursor + 1,
                           std::memory_order::memory_order_release);
    return true;
  }

private:
  std::atomic<std::size_t> producer_cursor_{0};
  std::atomic<std::size_t> consumer_cursor_{0};
  std::vector<T> buffer_;
};

template <typename T> using ChannelPtr = std::shared_ptr<Channel<T>>;

template <typename T> class ProducerSideChannel {
public:
  ProducerSideChannel(const ChannelPtr<T> &chnl) : channel{chnl} {}

  void push(T to_add) {
    poll();
    if (pending.empty() && channel->push(to_add)) {
      return;
    }
    pending.emplace_back(to_add);
  }

  void poll() {
    while (!pending.empty()) {
      if(channel->push(pending.front())) {
        pending.pop_front();
      }
      else break;
    }
  }

  ChannelPtr<T> channel;
  std::deque<T> pending;
};

template <typename T> struct Network {
  using Ref = std::reference_wrapper<Network<T>>;

  static void setUp(const std::vector<Ref> &subject,
                    std::size_t channels_capacity) {
    std::vector<std::vector<ChannelPtr<T>>> grid;
    grid.resize(subject.size());
    for (auto &row : grid) {
      for (std::size_t k = 0; k < subject.size(); ++k) {
        row.emplace_back(std::make_shared<Channel<T>>(channels_capacity));
      }
    }
    for (std::size_t k = 0; k < subject.size(); ++k) {
      for (std::size_t c = 0; c < subject.size(); ++c) {
        if (c == k) {
          continue;
        }
        auto channel = grid[k][c];
        subject[k].get().outgoing.emplace_back(channel);
      }
      for (std::size_t r = 0; r < subject.size(); ++r) {
        if (r == k) {
          continue;
        }
        auto channel = grid[r][k];
        subject[k].get().incoming.emplace_back(channel);
      }
    }
  }

  void push(const T &to_add) {
    for (auto &channel : outgoing) {
      channel.push(to_add);
    }
  }

  template <typename Process>
  void poll(std::size_t max_poll_per_channel_iterations, Process process) {
    for (auto &channel : this->outgoing) {
      channel.poll();
    }
    T tmp;
    for (auto &in : incoming) {
      for (std::size_t k = 0;
           k < max_poll_per_channel_iterations && in->poll(tmp); ++k) {
        process(tmp);
      }
    }
  }

protected:
  std::vector<ChannelPtr<T>> incoming;
  std::vector<ProducerSideChannel<T>> outgoing;
};
} // namespace mt_rrt
