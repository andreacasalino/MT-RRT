/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Planner.h>
#include <MT-RRT/ProblemDescription.h>
#include <MT-RRT/Solution.h>
#include <MT-RRT/TreeHandler.h>
#include <MT-RRT/TreeUtils.h>
#include <MT-RRT/extender/Types.h>

namespace mt_rrt::extender {
struct BidirSolution {
  std::vector<std::vector<float>> materialize() const;

  float cost() const {
    return byPassFront->cost2Root() + cost2Back + byPassBack->cost2Root();
  }

  const Node *byPassFront;
  const Node *byPassBack;
  float cost2Back;
};

template <typename TreeT> class Bidirectional : public ProblemAware {
public:
  using SolutionT = BidirSolution;

  TreeHandlerPtr<TreeT> front_handler;
  TreeHandlerPtr<TreeT> back_handler;

  Bidirectional(TreeHandlerPtr<TreeT> front,
                        TreeHandlerPtr<TreeT> back);

  void search_iteration(Solutions<BidirSolution> &solutions,
                        bool deterministic);

  void serializeTrees(
      std::vector<PlannerSolution::TreeSerialized> &recipient) const {
    recipient.emplace_back(serialize_tree(front_handler->nodes));
    recipient.emplace_back(serialize_tree(back_handler->nodes));
  }

  const Parameters &parameters() const { return front_handler->parameters; }

private:
  bool extension_state = false; // if true master is front, slave is back
};

//////////////////////////////////////////////////////////////////

template <typename TreeT>
Bidirectional<TreeT>::Bidirectional(TreeHandlerPtr<TreeT> front,
                                                    TreeHandlerPtr<TreeT> back)
    : ProblemAware(*front), front_handler{std::move(front)},
      back_handler{std::move(back)} {}

template <typename TreeT>
void Bidirectional<TreeT>::search_iteration(
    Solutions<BidirSolution> &solutions, bool deterministic) {
  extension_state = !extension_state;
  TreeHandler *master = front_handler.get(), *slave = back_handler.get();
  if (!extension_state) {
    std::swap(master, slave);
  }

  DescriptionAndParameters context =
      DescriptionAndParameters{problem(), parameters()};

  // extend master
  std::vector<float> sampled_state;
  if (!deterministic) {
    sampled_state = problem().sampler->sampleState();
  }
  View sampled_view =
      deterministic ? slave->nodes.front()->state() : View{sampled_state};
  auto master_steered = extend(sampled_view, *master, deterministic);
  if (!master_steered) {
    return;
  }

  const auto *master_added = master->internalize(master_steered->node);
  if (nullptr == master_added) {
    return;
  }

  auto slave_steered = extend(master_added->state(), *slave, false);
  if (!slave_steered) {
    return;
  }

  const auto *slave_added = slave->internalize(slave_steered->node);
  if (slave_steered->target_is_reached) {
    float connectCost = slave_steered->node.cost2Go();
    const Node *by_pass_begin = master_added;
    const Node *by_pass_end = slave_added;
    if (!extension_state) {
      std::swap(by_pass_begin, by_pass_end);
    }
    solutions.emplace_back(
        BidirSolution{by_pass_begin, by_pass_end, connectCost});
    return;
  }
}

} // namespace mt_rrt
