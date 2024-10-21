#include <Logger.h>

#include "ExtendTest.h"

namespace mt_rrt {

std::string make_log_tag(trivial::Kind kind) {
  std::string tag;
  switch (kind) {
  case trivial::Kind::Empty:
    tag = "empty";
    break;
  case trivial::Kind::NoSolution:
    tag = "no_solution";
    break;
  case trivial::Kind::SmallObstacle:
    tag = "one_obstacle";
    break;
  case trivial::Kind::Cluttered:
    tag = "cluttered";
    break;
  }
  return tag;
}

} // namespace mt_rrt
