#ifdef SHOW_PLANNER_PROGRESS
#include "Progress.h"

#include <iostream>
#include <sstream>
#include <thread>

namespace mt_rrt {
Progress::Progress() {
  std::stringstream buff;
  buff << "thread-" << std::this_thread::get_id() << ": ";
  label_ = buff.str();
}

Progress &Progress::get() {
  thread_local Progress res = Progress{};
  return res;
}

void Progress::reset() { iterations_ = 0; }

Progress &Progress::operator++() {
  std::scoped_lock lk{sharedMtx()};
  std::cout << label_ << ++iterations_ << std::endl;
  return *this;
}

std::mutex &Progress::sharedMtx() {
  static std::mutex res = std::mutex{};
  return res;
}
} // namespace mt_rrt

#endif
