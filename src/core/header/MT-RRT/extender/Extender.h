/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/extender/Types.h>
#include <MT-RRT/Solution.h>

#ifdef SHOW_PLANNER_PROGRESS
#include <MT-RRT/Progress.h>
#endif

namespace mt_rrt {
template<typename ExtenderImpl>
class Extender : public ExtenderImpl {
public:
  /** @brief Perform the specified number of estensions on the wrapped tree(s).
   * This function may be called multiple times, for performing batch of
   * extensions. All the solutions found while extending are saved and stored in
   * this object.
   * @param the number of extension to perform
   */
  std::size_t search() {
    const auto& pars = this->parameters();
    determinism_manager.emplace(problem().sampler->sampleSeed(),
                                pars.determinism);
    KeepSearchPredicate search_predicate =
        KeepSearchPredicate{pars.best_effort, pars.iterations.get(),
                            pars.expansion_strategy};

    std::size_t iter = 0;
    for (; search_predicate(iter); ++iter) {
      this->search_iteration(solutions);
      search_predicate.one_solution_was_found = !solutions.empty();
  #ifdef SHOW_PLANNER_PROGRESS
      ++Progress::get();
  #endif
    }

    return iter;
  }

  Solutions<typename ExtenderImpl::SolutionT> solutions;
};

} // namespace mt_rrt
