/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/ParallelFor.h>

namespace mt_rrt {
ParallelFor::~ParallelFor() {
  life.store(false, std::memory_order::memory_order_acquire);
  for (auto &worker : loops) {
    worker.join();
  }
}

ParallelFor::ParallelFor(const Threads &pool_size) {
  states.resize(pool_size.get() - 1);
  for (auto &state : states) {
    state.store(State::None, std::memory_order::memory_order_acquire);
  }
  for (std::size_t id = 1; id < pool_size.get(); ++id) {
    loops.emplace_back(std::bind(ParallelFor::worker_, std::ref(*this), id));
  }
}

void ParallelFor::worker_(std::size_t id) {
  while (true) {
    do {
      if (!life.load(std::memory_order::memory_order_acquire)) {
        return;
      }
      State expected = State::TaskReady;
      if (states[id - 1].compare_exchange_strong(
              expected, State::Processing,
              std::memory_order::memory_order_acquire)) {
        break;
      }
    } while (true);
    task_(id);
    states[id - 1].store(State::None, std::memory_order::memory_order_release);
  }
}
} // namespace mt_rrt