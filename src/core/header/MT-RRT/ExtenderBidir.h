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
class ExtenderBidirectional : public Extender {
public:
  ExtenderBidirectional(T first, T second, C &conn, const S &sampler);

  std::pair<T, T> trees_;

  struct Solution {
    // node from the first tree
    const Node &front;
    // node from the second tree
    const Node &back;
    float costTot;
    Positve cost2Bridge;

    mt_rrt::Solution materialize() const;
  };

  void extend() {
    ExtendResult res_master;
    if (shallThisBeDeterministic()) {
      res_master = this->Extender::extend<T, C, true>(
          slave_->root()->data().state, *master_, connector_);
    } else {
      sampler_.sampleState(sample_buffer_);
      res_master = this->Extender::extend<T, C, false>(
          std::span<const float>{sample_buffer_}, *master_, connector_);
    }

    const Node *master_added = std::visit(
        [](const auto &res_master) {
          if constexpr (std::is_same_v<decltype(res_master),
                                       const DeterministicTargetReached &>) {
            // new solution
            // TODO master reached slave root directly
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
      res_master = this->Extender::extend<T, C, true>(
          master_added->data().state, *slave_, connector_);

      if (const DeterministicTargetReached *trg_reached =
              std::get_if<DeterministicTargetReached>(&res_slave);
          trg_reached) {
        // new solution
        // TODO master reached slave root directly
      }
    }

    std::swap(master_, slave_);
  }

private:
  T *master_;
  T *slave_;

  C &connector_;
  const S &sampler_;
  std::vector<float> sample_buffer_;
  std::vector<Solution> solutions_;
};
} // namespace mt_rrt
