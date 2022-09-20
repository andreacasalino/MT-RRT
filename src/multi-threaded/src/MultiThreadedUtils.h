/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-multi-threaded/MultiThreadedPlanner.h>
#include <MT-RRT-multi-threaded/Synchronization.h>

#include <Extender.h>

#include <atomic>
#include <omp.h>

namespace mt_rrt {

Iterations
compute_balanced_numer_of_iterations(const Iterations &max_iterations,
                                     const Threads &threads);

using Extenders = std::vector<ExtenderPtr>;

// can be nullptr if none of the extenders found a solution
SolutionPtr get_best_solution(const Extenders &extenders);

void emplace_trees(PlannerSolution &recipient, const Extenders &extenders);

template <typename RootGenerator> TreePtr make_tree(RootGenerator gen) {
  TreePtr result = std::make_shared<Tree>();
  result->push_back(gen());
  return result;
}

std::size_t compute_batched_iterations(const Iterations &max_iterations,
                                       const Threads &threads,
                                       const SynchronizationDegree &synch);

template <typename Predicate>
void parallel_region(const Threads &threads, Predicate predicate) {
#pragma omp parallel num_threads(static_cast <int>(threads.get()))
  { predicate(); }
}
} // namespace mt_rrt
