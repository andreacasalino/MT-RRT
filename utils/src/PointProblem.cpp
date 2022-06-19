/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "PointProblem.h"

#include <MT-RRT-carpet/Error.h>

#include <algorithm>
#include <math.h>

namespace mt_rrt::utils {
namespace {
static const float STATE_BOX_DIAGONAL_LENGTH = 2.f * sqrtf(1.f);
}

const float PointConnector::STEER_DEGREE = STATE_BOX_DIAGONAL_LENGTH / 15.f;

PointConnector::PointConnector(const std::size_t size)
    : TunneledConnector(size) {
  if (size < 2) {
    throw Error{"Too small space size"};
  }
  obstacles.reset(new Boxes{});
  setSteerDegree(STEER_DEGREE);
}

namespace {
std::size_t validate_and_deduce_size(const Boxes &obstacles) {
  if (obstacles.empty()) {
    throw Error{"can't deduce space size"};
  }
  std::size_t size = obstacles.front().max_corner.size();
  for (const auto &box : obstacles) {
    if (box.min_corner.size() != size) {
      throw Error{"invalid box"};
    }
    if (box.max_corner.size() != size) {
      throw Error{"invalid box"};
    }
  }
  return size;
}
} // namespace

PointConnector::PointConnector(const Boxes &obstacles)
    : TunneledConnector(validate_and_deduce_size(obstacles)) {
  std::shared_ptr<Boxes> boxes;
  boxes.reset(new Boxes{obstacles});
  const auto size = getStateSpaceSize();
  for (auto &box : *boxes) {
    for (std::size_t k = 0; k < size; ++k) {
      const auto min = std::min(box.max_corner[k], box.min_corner[k]);
      const auto max = std::max(box.max_corner[k], box.min_corner[k]);
      box.min_corner[k] = min;
      box.max_corner[k] = max;
    }
  }
  this->obstacles = boxes;
  setSteerDegree(STEER_DEGREE);
}

PointConnector::PointConnector(const PointConnector &o)
    : TunneledConnector(o.getStateSpaceSize()) {
  this->obstacles = std::make_shared<Boxes>(*o.obstacles);
  setSteerDegree(STEER_DEGREE);
}

namespace {
enum class IntervalsCheck { Disjointed, EntirelyContained, Overlapping };
IntervalsCheck check_intervals(const State &segment_start,
                               const State &segment_end, const Box &box,
                               const std::size_t pos) {
  const float segment_min = std::min(segment_start[pos], segment_end[pos]);
  const float segment_max = std::max(segment_start[pos], segment_end[pos]);
  if ((segment_max < box.min_corner[pos]) ||
      (box.max_corner[pos] < segment_min)) {
    return IntervalsCheck::Disjointed;
  }
  if ((box.min_corner[pos] <= segment_min) &&
      (segment_max <= box.max_corner[pos])) {
    return IntervalsCheck::EntirelyContained;
  }
  return IntervalsCheck::Overlapping;
}
} // namespace

bool collides(const State &segment_start, const State &segment_end,
              const Box &box) {
  float segment_l1_norm = 0;
  bool all_entirely_contained = true;
  for (std::size_t k = 0; k < segment_start.size(); ++k) {
    switch (check_intervals(segment_start, segment_end, box, k)) {
    case IntervalsCheck::Disjointed:
      return false;
    case IntervalsCheck::Overlapping:
      all_entirely_contained = false;
      break;
    default:
      break;
    }
    float l1_norm = fabs(segment_start[k] - segment_end[k]);
    if (l1_norm > segment_l1_norm) {
      segment_l1_norm = l1_norm;
    }
  }

  if (all_entirely_contained) {
    return true;
  }

  if (segment_l1_norm < 1e-6f) {
    return true;
  }

  State mid_point;
  mid_point.reserve(segment_start.size());
  for (std::size_t k = 0; k < segment_start.size(); ++k) {
    mid_point.push_back(0.5f * (segment_start[k] + segment_end[k]));
  }
  return collides(segment_start, mid_point, box) ||
         collides(mid_point, segment_end, box);
}

namespace {
bool collides(const State &segment_start, const State &segment_end,
              const Boxes &boxes) {
  if (boxes.empty()) {
    return false;
  }
  return std::any_of(boxes.begin(), boxes.end(), [&](const Box &obstacle) {
    return collides(segment_start, segment_end, obstacle);
  });
}
} // namespace

bool PointConnector::checkAdvancement(const State &previous_state,
                                      const State &advanced_state) const {
  return collides(previous_state, advanced_state, *obstacles);
}
} // namespace mt_rrt::utils
