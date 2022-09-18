/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <ParallelFor.h>

namespace mt_rrt {
namespace {
void process__(const Jobs &jobs, const std::size_t thread_id,
               const std::size_t threads_num) {
  for (std::size_t k = thread_id; k < jobs.size(); k += threads_num) {
    jobs[k](thread_id);
  }
}

void wait_is_true(const std::atomic_bool &condition) {
    while (!condition.load()) {
    }
}
} // namespace

ParallelFor::ParallelFor(const Threads &pool_size) {
  for (std::size_t k = 1; k < pool_size.get(); ++k) {
    auto &worker = workers.emplace_back(std::make_unique<Worker>());
    struct WorkerContext {
      WorkerInfo &info;
      const std::size_t thread_id;
      const std::size_t threads_num;
    };
    std::atomic_bool started = false;
    worker->loop = std::make_unique<std::thread>(
        [context = WorkerContext{worker->info, k, pool_size.get()},
         &started]() {
          started = true;
          while (true) {
            wait_is_true(context.info.has_jobs);
            if (nullptr == context.info.jobs) {
              break;
            }
            process__(*context.info.jobs, context.thread_id,
                      context.threads_num);
            context.info.has_jobs.store(false);
            context.info.jobs = nullptr;
            context.info.jobs_processed.store(true);
          }
        });
    wait_is_true(started);
  }
}

void ParallelFor::process(const Jobs &jobs) {
  for (auto &worker : workers) {
    worker->info.jobs = &jobs;
    worker->info.has_jobs.store(true);
  }

  process__(jobs, 0, workers.size() + 1);

  for (auto &worker : workers) {
    wait_is_true(worker->info.jobs_processed);
    worker->info.jobs_processed.store(false);
  }
}

ParallelFor::~ParallelFor() {
  for (auto &worker : workers) {
    worker->info.jobs = nullptr;
    worker->info.has_jobs.store(true);
    worker->loop->join();
  }
}
} // namespace mt_rrt