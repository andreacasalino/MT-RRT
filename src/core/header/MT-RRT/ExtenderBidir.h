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
  const Node &front;
  // node from the second tree
  const Node &back;
  Positive cost2Bridge;
};

template <IsProblemDescription P, tree::HasBasicMethods T>
class ExtenderBidirectional
    : public Extender<P, ExtenderBidirectionalSolution> {
public:
  ExtenderBidirectional(Problem<P> &prblm, T first, T second)
      : Extender<P, ExtenderBidirectionalSolution>{prblm},
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
          a.cost2Root() + b.cost2Root() + cost2Bridge.get(), a, b,
          cost2Bridge});
    } else {
      this->pushSolution(ExtenderBidirectionalSolution{
          a.cost2Root() + b.cost2Root() + cost2Bridge.get(), b, a,
          cost2Bridge});
    }
  }
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <IsProblemDescription P, tree::HasBasicMethods T>
void ExtenderBidirectional<P, T>::extend() {
  RAIISwapper swapper{&master_, &slave_};

  ExtendResult res_master;
  if (this->determinismRegulator.shallThisBeDeterministic()) {
    res_master =
        this->template extend<T, true>(slave_->root()->data().state, *master_);
  } else {
    res_master = this->template extend<T, false>(this->sampleState(), *master_);
  }

  const Node *master_added = std::visit(
      [&](const auto &res_master) {
        if constexpr (std::is_same_v<decltype(res_master),
                                     const DeterministicTargetReached &>) {
          // new solution: master reaches directly the slave root
          this->pushSolution_(res_master->parent, slave_->root(),
                              res_master->cost2Go);
          return nullptr;
        }

        else if constexpr (std::is_same_v<decltype(res_master),
                                          const Steered &>) {
          return &res_master.added;

        }

        else {
          return nullptr;
        }
      },
      res_master);

  if (master_added) {
    ExtendResult res_slave;
    res_master =
        this->template extend<T, true>(master_added->data().state, *slave_);

    if (const DeterministicTargetReached *trg_reached =
            std::get_if<DeterministicTargetReached>(&res_slave);
        trg_reached) {
      // new solution
      this->pushSolution_(*master_added, trg_reached->parent,
                          trg_reached->cost2Go);
    }
  }
}
} // namespace mt_rrt
