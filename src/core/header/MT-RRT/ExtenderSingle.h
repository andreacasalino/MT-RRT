/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Extender.h>
#include <MT-RRT/Solution.h>

namespace mt_rrt {
struct ExtenderSingleSolution {
  float costTot;

  mt_rrt::Solution materialize() const;

  std::span<const float> target;
  const Node &pivot;
  Positive cost2Target;
};

template <IsProblemDescription P, tree::HasBasicMethods T>
class ExtenderSingle : public Extender<P, ExtenderSingleSolution> {
public:
  ExtenderSingle(Problem<P> &prblm, std::span<const float> target, T tree)
      : Extender<P, ExtenderSingleSolution>{prblm}, tree_{std::move(tree)},
        target_{target} {}

  T tree_;

  void extend();

  auto target() const { return target_; }

private:
  std::span<const float> target_;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <IsProblemDescription P, tree::HasBasicMethods T>
void ExtenderSingle<P, T>::extend() {
  ExtendResult res;
  if (determinismRegulator.shallThisBeDeterministic()) {
    res = this->Extender::extend<T, true>(target_, tree_);
  } else {
    res = this->Extender::extend<T, false>(sampleState(), tree_);
  }

  if (const DeterministicTargetReached *trg_reached =
          std::get_if<DeterministicTargetReached>(&res);
      trg_reached) {
    pushSolution(ExtenderSingleSolution{
        trg_reached->parent.cost2Root().get() + trg_reached->cost2Go, target_,
        trg_reached->parent, trg_reached->cost2Go});
  }
}
} // namespace mt_rrt
