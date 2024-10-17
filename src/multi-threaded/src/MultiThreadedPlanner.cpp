/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/MultiThreadedPlanner.h>

#include <thread>

namespace mt_rrt {
ProblemDescriptionCloner::ProblemDescriptionCloner(
    const ProblemDescriptionPtr &problem) {
  problem_description_copies.push_back(problem);
}

ProblemDescriptionPtr
ProblemDescriptionCloner::at(const std::size_t thread_id) {
  std::scoped_lock lock(problem_description_copies_mtx);
  resizeDescriptions(thread_id + 1);
  return problem_description_copies[thread_id];
}

void ProblemDescriptionCloner::resizeDescriptions(const std::size_t size) {
  while (problem_description_copies.size() < size) {
    const auto &to_clone = *problem_description_copies.front();
    ProblemDescription copy{to_clone.simmetry, to_clone.gamma,
                            to_clone.sampler->copy(),
                            to_clone.connector->copy()};
    problem_description_copies.push_back(
        std::make_shared<ProblemDescription>(std::move(copy)));
  }
}

Threads::Threads() : LowerLimited<std::size_t>(2, 4) {}

Threads::Threads(const std::size_t threads) : Threads() { set(threads); }

void MultiThreadedPlanner::setThreads(const Threads &threads_to_use) {
  threads.set(threads_to_use.get());
}

void MultiThreadedPlanner::setMaxThreads() {
  setThreads(std::thread::hardware_concurrency());
}
} // namespace mt_rrt
