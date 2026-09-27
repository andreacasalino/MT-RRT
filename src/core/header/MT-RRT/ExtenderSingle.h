/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Extender.h>
#include <MT-RRT/Sampler.h>
#include <MT-RRT/Solution.h>

namespace mt_rrt {
struct ExtenderSingleSolution {
  float costTot;

  mt_rrt::Solution materialize() const;

  std::span<const float> target;
  const Node &pivot;
  Positive cost2Target;
};

template <typename T, Connector C, Sampler S>
class ExtenderSingle : public Extender<ExtenderSingleSolution> {
public:
  ExtenderSingle(std::span<const float> target, T tree, C &conn,
                 const S &sampler);

  T tree_;

  void extend() {
    ExtendResult res;
    if (shallThisBeDeterministic()) {
      res = this->Extender::extend<T, C, true>(target_, tree_, connector_);
    } else {
      sampler_.sampleState(sample_buffer_);
      res = this->Extender::extend<T, C, false>(
          std::span<const float>{sample_buffer_}, tree_, connector_);
    }

    if (const DeterministicTargetReached *trg_reached =
            std::get_if<DeterministicTargetReached>(&res);
        trg_reached) {
      pushSolution(ExtenderSingleSolution{
          trg_reached->parent.cost2Root().get() + trg_reached->cost2Go, target_,
          trg_reached->parent, trg_reached->cost2Go});
    }
  }

  auto target() const { return target_; }

private:
  std::span<const float> target_;
  C &connector_;
  const S &sampler_;
  std::vector<float> sample_buffer_;
};
} // namespace mt_rrt
