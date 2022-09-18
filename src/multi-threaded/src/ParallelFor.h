/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-multi-threaded/MultiThreadedPlanner.h>

#include <algorithm>
#include <atomic>
#include <functional>
#include <thread>

namespace mt_rrt {
using Job = std::function<void(const std::size_t)>;
using Jobs = std::vector<Job>;

class ParallelFor {
public:
  ParallelFor(const Threads &pool_size);
  ~ParallelFor();

  template <typename Result, typename Iterator>
  std::vector<Result> process(
      const Iterator &begin, const Iterator &end,
      const std::function<void(Result &, const typename Iterator::value_type &,
                               const std::size_t)> &predicate) {
    std::vector<Result> results;
    results.reserve(workers.size() + 1);
    for (std::size_t k = 0; k < (workers.size() + 1); ++k) {
      results.emplace_back();
    }
    Jobs jobs;
    std::for_each(begin, end,
                  [&](const typename Iterator::value_type &element) {
                    jobs.emplace_back([&element, &predicate,
                                       &results](const std::size_t th_id) {
                      predicate(results[th_id], element, th_id);
                    });
                  });
    process(jobs);
    return results;
  }

  void process(const Jobs &jobs);

private:
  struct WorkerInfo {
    const Jobs *jobs = nullptr;
    std::atomic_bool has_jobs = false;
    std::atomic_bool jobs_processed = false;
  };

  struct Worker {
    WorkerInfo info;
    std::unique_ptr<std::thread> loop;
  };
  using WorkerPtr = std::unique_ptr<Worker>;
  std::vector<WorkerPtr> workers;
};
} // namespace mt_rrt
