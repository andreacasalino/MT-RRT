/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Solution.h>
#include <MT-RRT/extender/Types.h>
#include <MT-RRT/extender/Bidir.h>
#include <MT-RRT/extender/Single.h>

#ifdef SHOW_PLANNER_PROGRESS
#include <MT-RRT/Progress.h>
#endif

namespace mt_rrt {
namespace extender {
template <typename ExtenderImpl> class Extender : public ExtenderImpl {
public:
  template <typename... ARGS>
  Extender(ARGS &&...args)
      : ExtenderImpl{std::forward<ARGS>(args)...},
        determinismManager_{this->problem().sampler->sampleSeed(),
                            this->parameters().determinism} {}

  /** @brief Perform the specified number of estensions on the wrapped tree(s).
   * This function may be called multiple times, for performing batch of
   * extensions. All the solutions found while extending are saved and stored in
   * this object.
   * @param the number of extension to perform
   */
  std::size_t search() {
    const auto &pars = this->parameters();
    KeepSearchPredicate search_predicate = KeepSearchPredicate{
        pars.best_effort, pars.iterations.get(), pars.expansion_strategy};

    std::size_t iter = 0;
    for (; search_predicate(iter); ++iter) {
      this->search_iteration(solutions,
                             determinismManager_.doDeterministicExtension());
      search_predicate.one_solution_was_found.store(!solutions.empty(), std::memory_order::memory_order_release);
#ifdef SHOW_PLANNER_PROGRESS
      ++Progress::get();
#endif
    }

    return iter;
  }

  Solutions<typename ExtenderImpl::SolutionT> solutions;

protected:
  DeterminismRegulator determinismManager_;
};
}

template<typename TreeT>
using ExtenderSingle = extender::Extender<extender::Single<TreeT>>;

template<typename TreeT>
using ExtenderBidirectional = extender::Extender<extender::Bidirectional<TreeT>>;

} // namespace mt_rrt
