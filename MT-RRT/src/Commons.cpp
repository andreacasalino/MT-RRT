/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Commons.h"

namespace mt {
    std::vector<Problem*> make_battery(const std::vector<ProblemPtr>& problems) {
        std::vector<Problem*> battery;
        battery.reserve(problems.size());
        for (auto it = problems.begin(); it != problems.end(); ++it) {
            battery.emplace_back(it->get());
        }
        return battery;
    }

    template<typename Extender>
    void solveT(solver::SolutionInfo& info, solver::Parameters& param, Extender& ext) {
        ext.extend(param.Iterations_Max.get());
        info.iterations = ext.getIterationsDone();
        info.solution = ext.computeBestSolutionSequence();
    }

    void solveSingle(solver::SolutionInfo& info, solver::Parameters& param, const NodeState& end) {
        ExtSingle ext(param.Cumulate_sol, param.Deterministic_coefficient.get(), *info.trees.front(), end);
        solveT(info, param, ext);
    };

    void solveBidir(solver::SolutionInfo& info, solver::Parameters& param) {
        ExtBidir ext(param.Cumulate_sol, param.Deterministic_coefficient.get(), *info.trees[0], *info.trees[1]);
        solveT(info, param, ext);
    };

    template<typename Extender>
    void fillT(solver::SolutionInfo& info, solver::Parameters& param, const std::vector<Extender>& battery) {
        info.iterations = getIterationsDone(battery);
        info.solution = Extender::computeBestSolutionSequence(battery);
    }

    void fillSolutionInfo(solver::SolutionInfo& info, solver::Parameters& param, const std::vector<ExtSingle>& battery) {
        fillT(info, param, battery);
    };

    void fillSolutionInfo(solver::SolutionInfo& info, solver::Parameters& param, const std::vector<ExtBidir>& battery) {
        fillT(info, param, battery);
    };
}