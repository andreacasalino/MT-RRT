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
#include <iostream>

namespace mt::sample {
    enum StrategyType { Serial, MtQueryParall, MtSharedTree, MtLinkedTrees, MtMultiAgent };
    struct StrategyParameter {
        std::size_t iterations = 1000;
        std::size_t steerTrials = 1;
        float determinism = 0.1f;
        std::size_t threads = 0;
    };
    void setStrategy(solver::Solver& solver,  const StrategyType& type, const StrategyParameter& parameters);

    void addValues(arrayJSON& array, const float* data, const std::size_t& dataSize);

    void printData(const streamJSON& data, const std::string& fileName);

    class Results {
    public:
        Results() = default;

        // solve with all possible StrategyType
        Results(solver::Solver& solver, const NodeState& start, const NodeState& end, const StrategyParameter& parameters);

        void addResult(solver::Solver& solver, const StrategyType& mtStrategy, const solver::RRTStrategy& rrtStrategy);

        structJSON getJSON() const;

    private:
        std::map<StrategyType, std::map<solver::RRTStrategy, structJSON> > resultMatrix;
    };

    template<typename LoggableTrajManager>
    void logResults(const std::string& fileName, solver::Solver& solver, const Results& results) {
        mt::sample::structJSON log;
        solver.useProblem([&log](const mt::Problem& problem){
            log.addElement("problem", dynamic_cast<const LoggableTrajManager*>(problem.getTrajManager())->logDescription());
        });
        log.addEndl();
        log.addElement("results", results.getJSON());
        printData(log, fileName);
        std::cout << "use the python script Visualizer.py to visually check the results" << std::endl;
    }
}

#endif