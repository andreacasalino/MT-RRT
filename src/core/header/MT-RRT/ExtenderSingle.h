/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Extend.h>
#include <MT-RRT/Sampler.h>
#include <MT-RRT/Solution.h>

namespace mt_rrt {
template <typename T, Connector C, Sampler S>
class ExtenderSingle : public Extender {
public:
  ExtenderSingle(std::span<const float> target, T tree, C &conn,
                 const S &sampler);

  T extract() { return std::move(tree_); }

  struct Solution {
    Node &pivot;
    float costTot;
    Positve cost2Target;

    mt_rrt::Solution materialize() const;
  };

  void extend() {
    std::optional<DeterministicTargetReached> res;
    if (shallThisBeDeterministic()) {
      res = this->Extender::extend<T, C, true>(target_, tree_, connector_);
    } else {
      sampler_.sampleState(sample_buffer_);
      res = this->Extender::extend<T, C, false>(
          std::span<const float>{sample_buffer_}, tree_, connector_);
    }

    if (res.has_value()) {
      // new solution
      solutions_.emplace_back(
          Solution{res->parent, res->parent.cost2Root().get() + res->cost2Go,
                   res->cost2Go});
    }
  }

  auto target() const { return target_; }

  const auto &getSolutions() const { return solutions_; }

private:
  std::span<const float> target_;
  T tree_;
  C &connector_;
  const S &sampler_;
  std::vector<float> sample_buffer_;
  std::vector<Solution> solutions_;
};
} // namespace mt_rrt
