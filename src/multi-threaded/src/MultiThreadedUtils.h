/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Extender.h>
#include <MT-RRT/MultiThreadedPlanner.h>
#include <MT-RRT/Synchronization.h>

#include <atomic>
#include <omp.h>

namespace mt_rrt {

Iterations
compute_balanced_numer_of_iterations(const Iterations &max_iterations,
                                     const Threads &threads);

using Extenders = std::vector<ExtenderPtr>;

std::vector<std::vector<float>> get_best_solution(const Extenders &extenders);

void emplace_trees(PlannerSolution &recipient, const Extenders &extenders);

std::size_t compute_batched_iterations(const Iterations &max_iterations,
                                       const Threads &threads,
                                       const SynchronizationDegree &synch);

template <typename Predicate>
void parallel_region(const Threads &threads, Predicate &&predicate) {
#pragma omp parallel num_threads(static_cast <int>(threads.get()))
  { predicate(); }
}
} // namespace mt_rrt
