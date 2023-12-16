/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/ParallelFor.h>

namespace mt_rrt {
ParallelFor::~ParallelFor() {
  for (auto &worker : workers) {
    worker->context.life.store(false);
    worker->loop.join();
  }
}

ParallelFor::ParallelFor(const Threads &pool_size) {
  for (std::size_t k = 1; k < pool_size.get(); ++k) {
    auto *ctxt = &workers.emplace_back(std::make_unique<Worker>())->context;
    workers.back()->loop = std::thread{[ctxt = ctxt, id = k]() {
      while (ctxt->life.load()) {
        Phase expected{Phase::TaskReady};
        if (ctxt->phase.compare_exchange_strong(expected, Phase::Processing,
                                                std::memory_order_acquire)) {
          ctxt->task->scan(id);
          ctxt->task = nullptr;
          ctxt->phase.store(Phase::Done);
        }
      }
    }};
  }
}
} // namespace mt_rrt