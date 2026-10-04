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

namespace mt_rrt::trivial {
namespace {
static const float STATE_BOX_DIAGONAL_LENGTH = 2.f * sqrtf(1.f);
}

const float TrivialProblemConnector::STEER_DEGREE =
    STATE_BOX_DIAGONAL_LENGTH / 15.f;

TrivialProblemConnector::TrivialProblemConnector(BoxesPtr boxes,
                                                 SteerIterations steers)
    : EuclidianConnector<TrivialProblemChecker>{
          STEER_DEGREE, steers,
          std::make_unique<TrivialProblemChecker>(boxes)} {}

bool TrivialProblemChecker::check(std::span<const float> state) const {
  if (boxes_->empty()) {
    return true;
  }
  auto it = std::any_of(
      boxes_->begin(), boxes_->end(),
      [prev = geom::Point{previous_state}, adv = geom::Point{advanced_state}](
          const geom::Box &obstacle) { return obstacle.collides(prev, adv); });
  return it == boxes_->end();
}
} // namespace mt_rrt::trivial
