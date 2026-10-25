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
struct ExtenderBidirectionalSolution {
  float costTot;

  mt_rrt::Solution materialize() const;

  // node from the first tree
  const Node *front;
  // node from the second tree
  const Node *back;
  Positive cost2Bridge;
};

template <IsProblemDescription P, tree::HasBasicMethods T>
class ExtenderBidirectional
    : public Extender<P, ExtenderBidirectionalSolution> {
public:
  ExtenderBidirectional(Problem<P> &prblm, T first, T second)
      : Extender<P, ExtenderBidirectionalSolution>{prblm, first.root()
                                                              ->data()
                                                              .state.size()},
        trees_{std::make_pair(std::move(first), std::move(second))},
        master_{&trees_.first}, slave_{&trees_.second} {}

  std::pair<T, T> trees_;

  void extend();

private:
  struct RAIISwapper {
    T **m;
    T **s;

    ~RAIISwapper() {
      T *tmp = *m;
      *m = *s;
      *s = tmp;
    }
  };

  T *master_;
  T *slave_;

  void pushSolution_(const Node &a, const Node &b, Positive cost2Bridge) {
    if (master_ == &trees_.first) {
      this->pushSolution(ExtenderBidirectionalSolution{
          a.cost2Root() + b.cost2Root() + cost2Bridge.get(), &a, &b,
          cost2Bridge});
    } else {
      this->pushSolution(ExtenderBidirectionalSolution{
          a.cost2Root() + b.cost2Root() + cost2Bridge.get(), &b, &a,
          cost2Bridge});
    }
  }
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <IsProblemDescription P, tree::HasBasicMethods T>
void ExtenderBidirectional<P, T>::extend() {
  RAIISwapper swapper{&master_, &slave_};

  const Node *master_added{nullptr};

  ////////// master -> slave //////////
  if (this->determinismRegulator.shallThisBeDeterministic()) {
    ExtendResult res =
        this->template extend_<T, true>(slave_->root()->data().state, *master_);

    std::visit(
        [&](const auto &res) {
          if constexpr (std::is_same_v<decltype(res), const TargetReached &>) {
            // new solution: master reaches directly the slave root
            this->pushSolution_(*res.pivot, *slave_->root(), res.cost2Go);
          }

          else if constexpr (std::is_same_v<decltype(res), const Steered &>) {
            master_added = res.steered;
          }
        },
        res);
  }

  else {
    ExtendResult res =
        this->template extend_<T, false>(this->sampleState(), *master_);

    if (const Steered *as_steered = std::get_if<Steered>(&res); as_steered) {
      master_added = as_steered->steered;
    }
  }

  ////////// slave -> master //////////
  if (master_added) {
    ExtendResult res =
        this->template extend_<T, true>(master_added->data().state, *slave_);

    if (const TargetReached *trg_reached = std::get_if<TargetReached>(&res);
        trg_reached) {
      // new solution
      this->pushSolution_(*master_added, *trg_reached->pivot,
                          trg_reached->cost2Go);
    }
  }
}
} // namespace mt_rrt
