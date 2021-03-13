/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_COMMONS_H
#define MT_RRT_COMMONS_H

#include <math.h>
#include <solver/Strategy.h>
#include <ExtenderSingle.h>
#include <ExtenderBidir.h>

namespace mt {
    inline std::size_t computeBatchSize(const std::size_t& iterations, const double& reallCoeff, const std::size_t& threads) {
        return static_cast<size_t>(ceil(reallCoeff * static_cast<double>(iterations) / static_cast<double>(threads)));
    }

    void solveSingle(solver::SolutionInfo& info, solver::Parameters& param, const NodeState& end);

    void solveBidir(solver::SolutionInfo& info, solver::Parameters& param);

    void fillSolutionInfo(solver::SolutionInfo& info, solver::Parameters& param, const std::vector<ExtSingle>& battery);

    void fillSolutionInfo(solver::SolutionInfo& info, solver::Parameters& param, const std::vector<ExtBidir>& battery);
}

#endif