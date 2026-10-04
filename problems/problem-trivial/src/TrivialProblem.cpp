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

namespace mt_rrt::problem_trivial {
namespace {
static const float STATE_BOX_DIAGONAL_LENGTH = 2.f * sqrtf(1.f);
}

const float Connector::STEER_DEGREE = STATE_BOX_DIAGONAL_LENGTH / 15.f;

Connector::Connector(BoxesPtr boxes, SteerIterations steers)
    : EuclidianConnector<Checker>{STEER_DEGREE, steers,
                                  std::make_unique<Checker>(boxes)} {}

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
} // namespace mt_rrt::problem_trivial
