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

TrivialProblemConnector::TrivialProblemConnector(const geom::Boxes &obst)
    : TunneledConnector(2) {
  this->obstacles = std::make_shared<geom::Boxes>(obst);
  setSteerDegree(STEER_DEGREE);
}

TrivialProblemConnector::TrivialProblemConnector(
    const TrivialProblemConnector &o)
    : TunneledConnector(o.getStateSpaceSize()) {
  this->obstacles = std::make_shared<geom::Boxes>(*o.obstacles);
  setSteerDegree(STEER_DEGREE);
}

bool TrivialProblemConnector::checkAdvancement(
    const View &previous_state, const View &advanced_state) const {
  const auto &boxes = *this->obstacles;
  if (boxes.empty()) {
    return false;
  }
  return std::any_of(
      boxes.begin(), boxes.end(),
      [prev = geom::Point{previous_state}, adv = geom::Point{advanced_state}](
          const geom::Box &obstacle) { return obstacle.collides(prev, adv); });
}

std::shared_ptr<ProblemDescription>
TrivialProblemConnector::make(const std::optional<Seed> &seed,
                              const geom::Boxes &obstacles) {
  auto connector = std::make_unique<TrivialProblemConnector>(obstacles);

  std::vector<float> min_corner, max_corner;
  for (std::size_t k = 0; k < 2; ++k) {
    min_corner.push_back(-1.f);
    max_corner.push_back(1.f);
  }
  auto sampler = std::make_unique<HyperBox>(min_corner, max_corner, seed);

  std::shared_ptr<ProblemDescription> result;
  result.reset(new ProblemDescription{
      true, Positive<float>{10.f}, std::move(sampler), std::move(connector)});
  return result;
}
} // namespace mt_rrt::trivial
