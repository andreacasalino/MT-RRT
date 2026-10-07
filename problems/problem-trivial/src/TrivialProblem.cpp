/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <TrivialProblem.h>

#include <MT-RRT/Error.h>

#include <algorithm>
#include <math.h>

namespace mt_rrt::trivial_problem {
bool Checker::check_from_to(std::span<const float> from,
                            std::span<const float> to) const {
  if (boxes_->empty()) {
    return true;
  }
  return !std::any_of(
      boxes_->begin(), boxes_->end(),
      [from_point = geom::Point{from.data()},
       to_point = geom::Point{to.data()}](const geom::Box &obstacle) {
        return obstacle.collides(from_point, to_point);
      });
}

std::unique_ptr<Checker> make_checker(geom::Boxes obstacles) {
  return std::make_unique<Checker>(
      std::make_shared<geom::Boxes>(std::move(obstacles)));
}

} // namespace mt_rrt::trivial_problem

namespace mt_rrt {
void to_json(LogResult &j, const geom::Boxes &subject) {
  mt_rrt::to_json(j.addToScene("region"),
                  geom::Box{geom::PointAllocated{-1.f, -1.f},
                            geom::PointAllocated{1.f, 1.f}});
  for (const auto &box : subject) {
    j.addObstacle(box);
  }
}
} // namespace mt_rrt
