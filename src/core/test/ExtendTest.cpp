#include <LogResult.h>
#include <Logger.h>
#include <MiscConversions.h>
#include <TrivialProblemConversions.h>

#include "ExtendTest.h"

namespace mt_rrt {
void log_test_case(const std::string &tag, const std::string &title,
                   mt_rrt::Extender &subject) {
  LogResult res;
  to_json(res, static_cast<const trivial::TrivialProblemConnector &>(
                   *subject.problem().connector));
  for (const auto &solution : subject.getSolutions()) {
    res.addSolution(*solution);
  }
  for (auto &&tree : subject.dumpTrees()) {
    res.addTree(*tree);
  }
  Logger::get().add(tag, title, res.get());
}

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
