/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_STRATEGY_H
#define MT_RRT_STRATEGY_H

#include <solver/Solver.h>
#include <Limited.h>

namespace mt::solver {
    struct Parameters {
        bool Cumulate_sol = false;
        Limited<double>	Deterministic_coefficient = Limited<double>(0.01, 0.99, 0.2);
        LowerLimited<std::size_t> Iterations_Max = LowerLimited<std::size_t>(10, 1000);
    };

    class Strategy {
    public:
        virtual ~Strategy() = default;

        Strategy(const Strategy&) = delete;
        Strategy& operator=(const Strategy&) = delete;

        virtual std::unique_ptr<SolutionInfo> solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) = 0;

        inline void shareSolverData(std::shared_ptr<SolverData> solverData) { this->solverData = solverData; };

        inline void forgetSolverData() { this->solverData.reset(); };

        inline bool getCumulateFlag() const { return this->parameters.Cumulate_sol; };
        inline void setCumulateFlag(bool flag) { this->parameters.Cumulate_sol = flag; };

        inline Limited<double>& getDeterministicCoefficient() { return this->parameters.Deterministic_coefficient; };
        inline LowerLimited<std::size_t>& getIterationsMax() { return this->parameters.Iterations_Max; };

    protected:
        Strategy();

        Parameters parameters;

        std::shared_ptr<SolverData> solverData;
    };
}

#endif