/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Solution.h>
#include <MT-RRT/extender/Extender.h>
#include <MT-RRT/MultiThreadedPlanner.h>
#include <MT-RRT/Synchronization.h>

#include <atomic>
#include <omp.h>

namespace mt_rrt {

Iterations
compute_balanced_number_of_iterations(const Iterations &max_iterations,
                                      const Threads &threads);

template<typename ExtenderT>
using Extenders = std::vector<ExtenderT>;

template<typename T>
std::vector<std::vector<float>> materialize_best_in_extenders(const Extenders<T> &extenders) {
  using Solution = typename T::SolutionT;
  Solutions<Solution> solutions;
  for(const auto& extender : extenders) {
    solutions.insert(solutions.end(), extender.solutions.begin(), extender.solutions.end());
  }
  return materialize_best(solutions);
}

template<typename T>
void emplace_trees(PlannerSolution &recipient, Extenders<T> &extenders) {
  for (const auto &extender : extenders) {
    auto trees = extender.dumpTrees();
    for (auto &tree : trees) {
      recipient.trees.emplace_back(std::move(tree));
    }
  }
}

std::size_t compute_batched_iterations(const Iterations &max_iterations,
                                       const Threads &threads,
                                       const SynchronizationDegree &synch);

template <typename Predicate>
void parallel_region(const Threads &threads, Predicate &&predicate) {
#pragma omp parallel num_threads(static_cast <int>(threads.get()))
  { predicate(); }
}
} // namespace mt_rrt
