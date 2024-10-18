/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/TreeUtils.h>
#include <MT-RRT/extender/ExtenderBidir.h>

#include <algorithm>

namespace mt_rrt {
std::vector<std::vector<float>> BidirSolution::materialize() const {
  auto result = sequence_from_root(*byPassFront);
  auto result_to_append = sequence_from_root(*byPassBack);
  std::for_each(result_to_append.rbegin(), result_to_append.rend(),
                [&result](std::vector<float> &seq) {
                  result.emplace_back(std::move(seq));
                });
  return result;
}

float BidirSolution::cost() const {
  return byPassFront->cost2Root() + cost2Back + byPassBack->cost2Root();
}

ExtenderBidirectional::ExtenderBidirectional(TreeHandlerPtr front,
                                             TreeHandlerPtr back)
    : ProblemAware(*front), front_handler{std::move(front)},
      back_handler{std::move(back)} {}

void ExtenderBidirectional::search_iteration(
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
