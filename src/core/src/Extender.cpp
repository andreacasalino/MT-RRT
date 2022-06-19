/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-carpet/Error.h>

#include "Extender.h"

#include <list>
#include <math.h>

namespace mt_rrt {
std::vector<State> sequence_from_root(const Node &subject) {
  std::list<State> result = {subject.getState()};
  const auto *cursor = subject.getFatherInfo().father;
  for (; nullptr != cursor; cursor = cursor->getFatherInfo().father) {
    result.push_front(cursor->getState());
  }
  return std::vector<State>{result.begin(), result.end()};
}

void emplace_solution(PlannerSolution &recipient,
                      const SolutionPtr &solution_found) {
  if (nullptr == solution_found) {
    return;
  }
  recipient.solution = solution_found->getSequence();
}

void emplace_solution(PlannerSolution &recipient,
                      const Solutions &found_solutions) {
  if (found_solutions.empty()) {
    return;
  }
  emplace_solution(recipient, found_solutions.begin()->second);
}

std::vector<State> SingleSolution::getSequence() const {
  auto result = sequence_from_root(by_pass_node);
  result.push_back(target);
  return result;
}

std::vector<State> BidirSolution::getSequence() const {
  auto result = sequence_from_root(by_pass_begin);
  auto result_to_append = sequence_from_root(by_pass_end);
  result.insert(result.end(), result_to_append.rbegin(),
                result_to_append.rend());
  return result;
}

TreeHandler::TreeHandler(const NodePtr &root,
                         const ProblemDescriptionPtr &problem,
                         const Parameters &parameters)
    : ProblemAware(problem), parameters(parameters) {
  tree = std::make_shared<Tree>();
  if (nullptr == root) {
    throw Error{"invalid root"};
  }
  tree->push_back(root);
}

TreeHandler::TreeHandler(const TreePtr &tree,
                         const ProblemDescriptionPtr &problem,
                         const Parameters &parameters)
    : ProblemAware(problem), parameters(parameters) {
  if ((nullptr == tree) || (tree->empty())) {
    throw Error{"invalid tree"};
  }
  this->tree = tree;
};

DeterminismRegulator::DeterminismRegulator(const Seed &seed,
                                           const Determinism &determinism)
    : deterministic_rate_sampler(0, 1.f, seed),
      deterministic_rate_sampler_threshold(determinism.get()) {}

Extender::Extender(const TreeHandler &handler)
    : ProblemAware(handler), parameters(handler.parameters) {}

std::size_t Extender::search() {
  determinism_manager = std::make_unique<DeterminismRegulator>(
      problem().sampler->sampleSeed(), parameters.determinism);

  KeepSearchPredicate search_predicate =
      KeepSearchPredicate{parameters.best_effort, parameters.iterations.get(),
                          parameters.expansion_strategy};

  std::size_t iter = 0;
  for (; search_predicate(iter); ++iter) {
    search_iteration();
    search_predicate.one_solution_was_found = !solutions.empty();
#ifdef SHOW_PLANNER_PROGRESS
    ++PLANNER_PROGRESS_SINGLETON;
#endif
  }

  return iter;
}

namespace {
bool contains(const Node &to_steer_candidate, const State &target,
              const TreeHandler::DeterministicSteerRegister &det_register) {
  auto det_register_it = det_register.find(&to_steer_candidate);
  if (det_register_it == det_register.end()) {
    return false;
  }
  return det_register_it->second.find(&target) != det_register_it->second.end();
}

std::optional<SteerResult> extend(const State &target,
                                  TreeHandler &tree_handler,
                                  const bool is_deterministic) {
  auto *nearest = tree_handler.nearestNeighbour(target);
  auto &det_register = tree_handler.deterministic_steer_register;
  if (!nearest ||
      (is_deterministic && contains(*nearest, target, det_register))) {
    return std::nullopt;
  }
  if (is_deterministic) {
    auto det_register_it = det_register.find(nearest);
    if (det_register_it == det_register.end()) {
      det_register_it =
          det_register.emplace(nearest, std::unordered_set<const State *>{})
              .first;
    }
    det_register_it->second.emplace(&target);
  }
  return tree_handler.problem().connector->steer(
      *nearest, target, tree_handler.parameters.steer_trials);
}

std::optional<SteerResult> extend_star(const State &target,
                                       TreeHandler &tree_handler,
                                       const bool is_deterministic,
                                       Rewires &rewires) {
  auto maybe_steered = extend(target, tree_handler, is_deterministic);
  if (!maybe_steered) {
    return std::nullopt;
  }
  const auto near_set = tree_handler.nearSet(*maybe_steered->steered_node);

  rewires = compute_rewires(*maybe_steered->steered_node, near_set,
                            DescriptionAndParameters{tree_handler.problem(),
                                                     tree_handler.parameters});
  return maybe_steered;
}
} // namespace

ExtenderSingle::ExtenderSingle(TreeHandlerPtr handler, const State &target)
    : Extender(*handler), target(target) {
  tree_handler = std::move(handler);
}

ExtenderSingle::ExtendResult ExtenderSingle::extend_tree(bool toward_target) {
  std::optional<State> sampled_state;
  if (!toward_target) {
    sampled_state.emplace(problem().sampler->sampleState());
  }
  const State &target_state = toward_target ? target : sampled_state.value();

  ExtendResult result;
  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single:
    result.steer_result = extend(target_state, *tree_handler, toward_target);
    break;
  case ExpansionStrategy::Star:
    result.rewires.emplace();
    result.steer_result = extend_star(target_state, *tree_handler,
                                      toward_target, *result.rewires);
    break;
  default:
    throw Error{"Trying to use ExtenderSingle with a bidirectional strategy"};
    break;
  }
  return result;
}

void ExtenderSingle::search_iteration() {
  const auto toward_target = determinism_manager->doDeterministicExtension();

  auto extend_result = extend_tree(toward_target);
  auto &steered = extend_result.steer_result;
  if (!steered) {
    return;
  }

  if (toward_target && steered->target_is_reached) {
    const float solution_cost = steered->steered_node->cost2Root();
    auto solution = std::make_unique<SingleSolution>(
        *steered->steered_node->getFatherInfo().father, target);
    solutions.emplace(solution_cost, std::move(solution));
    return;
  }

  auto *added = tree_handler->add(std::move(*steered->steered_node));
  if ((nullptr == added) || (!extend_result.rewires)) {
    return;
  }
  tree_handler->applyRewires(*added, *extend_result.rewires);
}

ExtenderBidirectional::ExtenderBidirectional(TreeHandlerPtr front,
                                             TreeHandlerPtr back)
    : Extender(*front) {
  front_handler = std::move(front);
  back_handler = std::move(back);
}

void ExtenderBidirectional::search_iteration() {
  extension_state = !extension_state;
  TreeHandler *master = front_handler.get(), *slave = back_handler.get();
  if (!extension_state) {
    std::swap(master, slave);
  }

  DescriptionAndParameters context =
      DescriptionAndParameters{problem(), parameters};

  const auto deterministic_extension =
      determinism_manager->doDeterministicExtension();

  // extend master
  std::optional<State> sampled_state;
  if (!deterministic_extension) {
    sampled_state = problem().sampler->sampleState();
  }
  const State &master_target_state = deterministic_extension
                                         ? slave->tree->front()->getState()
                                         : sampled_state.value();
  auto master_steered =
      extend(master_target_state, *master, deterministic_extension);
  if (!master_steered) {
    return;
  }

  const auto *master_added =
      master->add(std::move(*master_steered->steered_node));
  if (nullptr == master_added) {
    return;
  }

  auto slave_steered = extend(master_added->getState(), *slave, false);
  if (!slave_steered) {
    return;
  }

  if (slave_steered->target_is_reached) {
    const float solution_cost =
        master_added->cost2Root() + slave_steered->steered_node->cost2Root();

    const Node *by_pass_begin = master_added;
    const Node *by_pass_end =
        slave_steered->steered_node->getFatherInfo().father;
    if (!extension_state) {
      std::swap(by_pass_begin, by_pass_end);
    }

    auto solution =
        std::make_unique<BidirSolution>(*by_pass_begin, *by_pass_end);
    solutions.emplace(solution_cost, std::move(solution));
    return;
  }
  slave->add(std::move(*slave_steered->steered_node));
}
} // namespace mt_rrt
