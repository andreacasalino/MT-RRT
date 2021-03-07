/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_LOGGER_H
#define MT_RRT_SAMPLE_LOGGER_H

#include <string>
#include <Solver.h>
#include <JSONstream.h>
#include <map>

namespace mt::sample {
    void addValues(arrayJSON& array, const float* data, const std::size_t& dataSize);

    void printData(const streamJSON& data, const std::string& fileName);

    class Results {
    public:
        Results() = default;

        // solve with all possible strategies
        Results(Solver& solver, const NodeState& start, const NodeState& end, const std::size_t& threads, const bool& interpolateSolution = false);

        void addResult(Solver& solver, const Solver::MTStrategy& mtStrategy, const Solver::RRTStrategy& rrtStrategy, const bool& interpolateSolution = false);

        structJSON getJSON() const;

    private:
        std::map<Solver::MTStrategy, std::map<Solver::RRTStrategy, structJSON> > resultMatrix;
    };
}

#endif