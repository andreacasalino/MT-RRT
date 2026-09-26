/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/Trajectory.h>

namespace mt_rrt {
template <typename Constraints> class EucledianConnector;

template <typename Constraints> class EucledianSegment {
public:
  EucledianSegment(EucledianConnector<Constraints> &source,
                   std::span<const float> start, std::span<const float> end);

  Positive minCost2Go() const;

  /**
   * @brief tunneled advancement see TODO Chapter
   */
  std::optional<TraverseResult> traverse();

private:
  EucledianConnector<Constraints> &source_;
  std::span<const float> start_;
  std::span<const float> end_;
};

template <typename Constraints> class EucledianConnector {
public:
  using trajectory_type = EucledianSegment<Constraints>;

  friend class trajectory_type;

  EucledianConnector(float quantized_advancement, Constraints constraints);

  std::optional<trajectory_type> makeTrajectory(std::span<const float> start,
                                                std::span<const float> end);

  std::unique_ptr<EucledianConnector> copy() const {
    return std::make_unique<EucledianConnector>(quantized_advancement_,
                                                constraints_);
  }

private:
  float quantized_advancement_;
  Constraints constraints_;
  std::vector<float> advance_buffer_;
};
} // namespace mt_rrt
