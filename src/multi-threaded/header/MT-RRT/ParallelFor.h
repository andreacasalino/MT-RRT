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

namespace mt_rrt {
class ParallelFor {
public:
  ParallelFor(const Threads &pool_size);
  ~ParallelFor();

  template <typename T, typename Pred>
  void process(const std::vector<T> &elements, Pred &&pred) {
    ForEachT<T> forEach{workers.size() + 1, elements, std::forward<Pred>(pred)};
    for (auto &worker : workers) {
      worker->context.task = &forEach;
      worker->context.phase.store(Phase::TaskReady);
    }
    forEach.scan(0);
    for (auto &worker : workers) {
      do {
        Phase expected{Phase::Done};
        if (worker->context.phase.compare_exchange_strong(
                expected, Phase::None, std::memory_order_release)) {
          break;
        }
      } while (true);
    }
  }

  std::size_t size() const { return workers.size() + 1; }

private:
  struct ForEach {
    virtual ~ForEach() = default;
    virtual void scan(std::size_t threadId) = 0;
  };

  template <typename T> struct ForEachT : ForEach {
    template <typename Pred>
    ForEachT(std::size_t threads, const std::vector<T> &elements, Pred &&pred)
        : threads{threads}, elements{elements}, pred{std::forward<Pred>(pred)} {
    }

    std::size_t threads;
    const std::vector<T> &elements;
    std::function<void(const T &, std::size_t)> pred;

    void scan(std::size_t threadId) final {
      for (std::size_t k = threadId; k < elements.size(); k += threads) {
        pred(elements[k], threadId);
      }
    }
  };

  enum class Phase { None, TaskReady, Processing, Done };

  struct WorkerContext {
    std::atomic_bool life = true;
    std::atomic<Phase> phase = Phase::None;
    ForEach *task = nullptr;
  };

  struct Worker {
    WorkerContext context;
    std::thread loop;
  };
  std::vector<std::unique_ptr<Worker>> workers;
};
} // namespace mt_rrt
