/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <TreeCore.h>
#include <list>
#include <sampler/engine/UniformEngine.h>
#include <set>

namespace mt_rrt {
/**
 * @brief Someone able to extend one or two connected search trees
 */
template <typename Solution> class Extender {
public:
  virtual ~Extender() = default;

  /**
   * @brief Perform the specified number of estensions on the controlled
   * tree(s). This function may be called multiple times, for performing batch
   * of extensions. All the solutions found while extending are saved and stored
   * into this object.
   * @param the number of extension to perform
   */
  virtual void extend(const std::size_t &Iterations) = 0;

  /**
   * @brief Get the extensions done so far (all extend() called so far are
   * accounted).
   */
  inline std::size_t getIterationsDone() const { return this->iterationsDone; };

  /**
   * @brief Get the population of solutions found so far
   */
  inline const std::set<Solution> &getSolutions() const {
    return this->solutionsFound;
  };

  /**
   * @return the sequence pertaining to the best found solution.
   * In case no solution was found, an empty vector is returned.
   */
  std::vector<NodeState> computeBestSolutionSequence() const {
    if (this->solutionsFound.empty()) {
      return {};
    }
    return this->computeSolutionSequence(*this->solutionsFound.begin());
  };

  /**
   * @return the sequence pertaining to the best solution found,
   * among all the ones stored in the passed extenders
   */
  template <typename ExtT>
  static std::vector<NodeState>
  computeBestSolutionSequence(const std::vector<ExtT> extenders) {
    std::set<Solution> solutions;
    for (auto itE = extenders.begin(); itE != extenders.end(); ++itE) {
      for (auto itSol = itE->solutionsFound.begin();
           itSol != itE->solutionsFound.end(); ++itSol) {
        if (solutions.find(*itSol) == solutions.end()) {
          solutions.emplace(*itSol);
        }
      }
    }
    if (solutions.empty())
      return {};
    return extenders.front().computeSolutionSequence(*solutions.begin());
  };

  inline bool isCumulating() const { return this->cumulateSolutions; };

protected:
  Extender(const bool &cumulateSolutions,
           const double &deterministicCoefficient)
      : cumulateSolutions(cumulateSolutions),
        deterministicCoefficient(deterministicCoefficient){};

  virtual std::vector<NodeState>
  computeSolutionSequence(const Solution &sol) const = 0;

  sampling::UniformEngine randEngine;
  /**
   * @brief when set true, the extension process is not arrested when a
   * solution is found, but the extender keeps on trying to find also additional
   * solutions.
   */
  const bool cumulateSolutions;
  const double deterministicCoefficient;
  std::size_t iterationsDone = 0;
  std::set<Solution> solutionsFound;
};

std::vector<NodeState> convert(const std::list<const NodeState *> nodes);

TreeCore *convert(Tree *t);

/**
 * @return the sum of extensions done by all the passed extenders
 */
template <typename Extender>
std::size_t getIterationsDone(const std::vector<Extender> &battery) {
  std::size_t iter = 0;
  for (auto it = battery.begin(); it != battery.end(); ++it) {
    iter += it->getIterationsDone();
  }
  return iter;
};

#ifdef SHOW_PROGRESS
class ProgressPrinter {
public:
  static void show(const std::size_t &iter);

private:
  static std::mutex coutMtx;
};
#endif
} // namespace mt_rrt
