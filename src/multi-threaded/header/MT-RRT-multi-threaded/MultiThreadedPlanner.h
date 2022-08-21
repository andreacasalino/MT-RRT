/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/Planner.h>

#include <mutex>
#include <optional>

namespace mt_rrt {
class ProblemDescriptionCloner {
public:
  virtual ~ProblemDescriptionCloner() = default;

protected:
  ProblemDescriptionCloner(const ProblemDescriptionPtr &problem);

  ProblemDescriptionPtr problemAt(const std::size_t thread_id);

  const std::vector<ProblemDescriptionPtr> &getAllDescriptions() const {
    return problem_description_copies;
  }

  void resizeDescriptions(const std::size_t size);

private:
  std::mutex problem_description_copies_mtx;
  std::vector<ProblemDescriptionPtr> problem_description_copies;
};

class Threads : public LowerLimited<std::size_t> {
public:
  Threads();

  Threads(const std::size_t threads);
};

class MultiThreadedPlanner : public Planner, public ProblemDescriptionCloner {
public:
  template <typename... Args>
  MultiThreadedPlanner(Args... args)
      : Planner(std::forward<Args>(args)...),
        ProblemDescriptionCloner(problemPtr()) {}

  void setThreads(const Threads &threads_to_use);
  // max number of available threads is used
  void setMaxThreads();
  std::size_t getThreads() const { return threads.get(); };

private:
  Threads threads;
};
} // namespace mt_rrt
