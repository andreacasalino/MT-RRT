#ifdef SHOW_PLANNER_PROGRESS
#pragma once

#include <mutex>
#include <string>

namespace mt_rrt {
/**
 * @brief Debug tool that shows the planner advancement in the console.
 */
class Progress {
public:
  static Progress &get();

  void reset();

  Progress &operator++();

private:
  Progress();

  static std::mutex &sharedMtx();

  std::string label_;
  std::size_t iterations_ = 0;
};
} // namespace mt_rrt

#endif
