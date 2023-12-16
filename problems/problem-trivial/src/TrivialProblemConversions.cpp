/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MiscConversions.h>
#include <TrivialProblemConversions.h>

namespace mt_rrt {
void to_json(LogResult &j, const trivial::TrivialProblemConnector &subject) {
  to_json(j.addToScene("region"), geom::Box{{-1.f, -1.f}, {1.f, 1.f}});
  for (const auto &box : subject.getBoxes()) {
    j.addObstacle(box);
  }
}
} // namespace mt_rrt
