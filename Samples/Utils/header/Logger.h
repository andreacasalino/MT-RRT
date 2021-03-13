/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_LOGGER_H
#define MT_RRT_SAMPLE_LOGGER_H

#include <string>
#include <solver/Solver.h>
#include <solver/Strategy.h>
#include <JSONstream.h>
#include <map>

namespace mt::sample {
    enum StrategyType { Serial, MtQueryParall, MtSharedTree, MtLinkedTrees, MtMultiAgent };

    std::unique_ptr<solver::Strategy> make_strategy(const StrategyType& type);

    void addValues(arrayJSON& array, const float* data, const std::size_t& dataSize);

    void printData(const streamJSON& data, const std::string& fileName);

    class Results {
    public:
        Results() = default;

        // solve with all possible StrategyType
        Results(solver::Solver& solver, const NodeState& start, const NodeState& end, const std::size_t& threads, const bool& interpolateSolution = false);

        void addResult(solver::Solver& solver, const StrategyType& mtStrategy, const solver::RRTStrategy& rrtStrategy, const bool& interpolateSolution = false);

        structJSON getJSON() const;

    private:
        std::map<StrategyType, std::map<solver::RRTStrategy, structJSON> > resultMatrix;
    };
}

#endif