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

  void extend() {
    std::optional<DeterministicTargetReached> res;
    if (shallThisBeDeterministic()) {
      res = this->Extender::extend(target_, tree_, connector_);
    } else {
      sampler_.sampleState(sample_buffer_);
      res = this->Extender::extend(std::span<const float>{sample_buffer_},
                                   tree_, connector_);
    }

    if (res.has_value()) {
      // new solution
    }
  }

  auto target() const { return target_; }

private:
  std::span<const float> target_;
  T tree_;
  C &connector_;
  const S &sampler_;
  std::vector<float> sample_buffer_;
};
} // namespace mt_rrt
