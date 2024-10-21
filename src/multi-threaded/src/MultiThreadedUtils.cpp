/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "MultiThreadedUtils.h"

namespace mt_rrt {
Iterations
compute_balanced_number_of_iterations(const Iterations &max_iterations,
                                     const Threads &threads) {
  std::size_t result = max_iterations.get() / threads.get();
  result = std::max<std::size_t>(1, result);
  return result;
}

std::size_t compute_batched_iterations(const Iterations &max_iterations,
                                       const Threads &threads,
                                       const SynchronizationDegree &synch) {
  const auto max_iterations_per_thread =
      compute_balanced_number_of_iterations(max_iterations, threads).get();

  const auto iterations_per_cycle_float =
      std::floor(max_iterations_per_thread * synch.get());

  return std::max<std::size_t>(
      static_cast<std::size_t>(iterations_per_cycle_float), 1);
}
} // namespace mt_rrt
