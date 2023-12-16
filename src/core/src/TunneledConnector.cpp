/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/TunneledConnector.h>

#include <math.h>

namespace mt_rrt {
float euclidean_distance(const View &a, const View &b) {
  float result = 0;
  for (std::size_t k = 0; k < a.size; ++k) {
    result += powf(a.data[k] - b.data[k], 2.f);
  }
  return sqrtf(result);
}

TunneledConnector::TunneledConnector(const std::size_t size)
    : space_size(size) {
  if (size < 2) {
    throw Error{"Too small space size"};
  }
};

TunneledConnector::TunneledConnector(const TunneledConnector &o)
    : TunneledConnector(o.space_size) {
  steer_degree.set(o.steer_degree.get());
}

float TunneledConnector::minCost2Go(const View &start, const View &end) const {
  return euclidean_distance(start, end);
}

TunneledConnector::Line::Line(const View &start, const View &end,
                              const TunneledConnector &caller)
    : caller(caller), start(start), actual(start.convert()), target(end) {}

Trajectory::AdvanceInfo TunneledConnector::Line::advance() {
  float remaining_distance = euclidean_distance(actual, target);
  if (remaining_distance <= caller.steer_degree.get()) {
    if (caller.checkAdvancement(actual, target)) {
      return Trajectory::AdvanceInfo::blocked;
    }
    actual = target.convert();
    return Trajectory::AdvanceInfo::targetReached;
  }
  advanced = actual;
  for (std::size_t k = 0; k < actual.size(); ++k) {
    advanced[k] += caller.steer_degree.get() * (target.data[k] - actual[k]) /
                   remaining_distance;
  }
  auto result = caller.checkAdvancement(actual, advanced)
                    ? Trajectory::AdvanceInfo::blocked
                    : Trajectory::AdvanceInfo::advanced;
  if (result == Trajectory::AdvanceInfo::advanced) {
    actual = std::move(advanced);
  }
  return result;
}

float TunneledConnector::Line::getCumulatedCost() const {
  return euclidean_distance(start, actual);
}

TrajectoryPtr TunneledConnector::getTrajectory(const View &start,
                                               const View &end) const {
  return std::make_unique<Line>(start, end, *this);
}
} // namespace mt_rrt
