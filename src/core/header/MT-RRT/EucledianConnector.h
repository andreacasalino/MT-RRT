/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>

namespace mt_rrt {
template <typename Constraints> class EucledianConnector {
public:
  EucledianConnector(float quantized_advancement, Constraints constraints);

  // euclidean distance in the state space
  Positive minCost2Go(std::span<const float> start,
                      std::span<const float> target);

  std::optional<TraverseResult> steer(std::span<const float> start,
                                      std::span<const float> target,
                                      std::vector<float> &reached);

private:
  float quantized_advancement_;
  Constraints constraints_;
};
} // namespace mt_rrt
