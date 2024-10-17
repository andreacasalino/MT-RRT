/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/MultiThreadedPlanner.h>

#include <algorithm>
#include <atomic>
#include <functional>
#include <thread>
#include <type_traits>

namespace mt_rrt {
class ParallelFor {
public:
  ParallelFor(const Threads &pool_size);
  ~ParallelFor();

  template <typename T, typename Pred>
  void process(const std::vector<T> &elements, Pred pred) {
    task_ = [&pred, &elements, workers = loops.size()](std::size_t worker_id) {
      for (std::size_t k = worker_id; k < elements.size(); k += workers) {
        if constexpr (std::is_pointer_v<T>) {
          pred(elements[k], worker_id);
        } else {
          pred(std::ref(elements[k]), worker_id);
        }
      }
    };
    for (auto &state : states) {
      state.store(State::TaskReady, std::memory_order::memory_order_acquire);
    }
    task_(0);
    for (auto &state : states) {
      while (state.load(std::memory_order::memory_order_release) !=
             State::None) {
      }
    }
  }

  std::size_t size() const { return loops.size() + 1; }

private:
  void worker_(std::size_t id);

  std::atomic_bool life{true};
  std::function<void(std::size_t)> task_;

  enum class State { None, TaskReady, Processing };
  std::vector<std::atomic<State>> states;
  std::vector<std::thread> loops;
};
} // namespace mt_rrt
