/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-core/TunneledConnector.h>

#include <math.h>

namespace mt_rrt {
float euclidean_distance(const State &a, const State &b) {
  float result = 0;
  for (std::size_t k = 0; k < a.size(); ++k) {
    result += powf(a[k] - b[k], 2.f);
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

float TunneledConnector::minCost2Go(const State &start,
                                    const State &end) const {
  return euclidean_distance(start, end);
}

TrajectoryPtr TunneledConnector::getTrajectory(const State &start,
                                               const State &end) const {
  return std::make_unique<Line>(start, end, *this);
}

TunneledConnector::Line::Line(const State &start, const State &end,
                              const TunneledConnector &caller)
    : caller(caller), start(start), attual(start), target(end) {}

AdvanceInfo TunneledConnector::Line::advance() {
  const float remaining_distance = euclidean_distance(attual, target);
  if (remaining_distance <= caller.steer_degree.get()) {
    if (caller.checkAdvancement(attual, target)) {
      return AdvanceInfo::blocked;
    }
    attual = target;
    return AdvanceInfo::targetReached;
  }
  State next_state = attual;
  for (std::size_t k = 0; k < attual.size(); ++k) {
    next_state[k] += caller.steer_degree.get() * (target[k] - attual[k]) /
                     remaining_distance;
  }
  auto const result = caller.checkAdvancement(attual, next_state)
                          ? AdvanceInfo::blocked
                          : AdvanceInfo::advanced;
  attual = std::move(next_state);
  return result;
}

float TunneledConnector::Line::getCumulatedCost() const {
  return euclidean_distance(start, attual);
}
} // namespace mt_rrt
