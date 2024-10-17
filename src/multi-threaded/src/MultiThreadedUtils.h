/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/MultiThreadedPlanner.h>
#include <MT-RRT/Solution.h>
#include <MT-RRT/Synchronization.h>
#include <MT-RRT/extender/Extender.h>

#include <atomic>
#include <thread>
#include <functional>

namespace mt_rrt {

Iterations
compute_balanced_number_of_iterations(const Iterations &max_iterations,
                                      const Threads &threads);

template <typename ExtenderT> using Extenders = std::vector<ExtenderT>;

template <typename ExtenderT>
void serializeTrees(const Extenders<ExtenderT> &extenders,
                    const Parameters &parameters, PlannerSolution &recipient) {
  if (parameters.dumpTrees) {
    for (const auto &ext : extenders) {
      ext.serializeTrees(recipient.trees);
    }
  }
}

template <typename T>
std::vector<std::vector<float>>
materialize_best_in_extenders(const Extenders<T> &extenders) {
  using Solution = typename T::SolutionT;
  Solutions<Solution> solutions;
  for (const auto &extender : extenders) {
    solutions.insert(solutions.end(), extender.solutions.begin(),
                     extender.solutions.end());
  }
  return materialize_best(solutions);
}

template <typename T>
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
  std::vector<std::thread> workers;
  for (std::size_t th_id = 0; th_id < threads.get(); ++th_id) {
    workers.emplace_back(std::bind(predicate, th_id));
  }
  for (auto &w : workers) {
    w.join();
  }
}

template<typename TreeT>
Extenders<ExtenderSingle<TreeT>> make_single_extenders(std::vector<TreeHandlerPtr<TreeT>> trees, const std::vector<float>& target) {
  Extenders<ExtenderSingle<TreeT>> res;
  for(auto& tree : trees) {
    res.emplace_back(std::move(tree), target);
  }
  return res;
}

template<typename TreeT>
Extenders<ExtenderBidirectional<TreeT>> make_bidirectional_extenders(std::vector<TreeHandlerPtr<TreeT>> front, std::vector<TreeHandlerPtr<TreeT>> back) {
  if(front.size() != back.size()) {
    throw Error{"front and back trees must have the same size"};
  }
  Extenders<ExtenderBidirectional<TreeT>> res;
  for(std::size_t k=0; k<front.size(); ++k) {
    res.emplace_back( std::move(front[k]), std::move(back[k]) );
  }
  return res;
}

} // namespace mt_rrt
