/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Logger.h>
#include <fstream>
#include <Error.h>
#include <Interpolator.h>

namespace mt::sample {
    void addValues(arrayJSON& array, const float* data, const std::size_t& dataSize) {
        for (std::size_t k = 0; k < dataSize; ++k) {
            array.addElement(Number<float>(data[k]));
        }
    };

    void printData(const streamJSON& data, const std::string& fileName) {
        std::ofstream f(fileName);
        if (!f.is_open()) {
            throw Error("invalid log file");
        }
        f << data.str();
    }

    std::string tostring(const Solver::RRTStrategy& rrtStrategy) {
        switch (rrtStrategy) {
        case Solver::RRTStrategy::Single:
            return "Single";
            break;
        case Solver::RRTStrategy::Bidir:
            return "Bidir";
            break;
        case Solver::RRTStrategy::Star:
            return "Star";
            break;
        default:
            throw Error("unknown");
            break;
        }
        throw Error("unknown");
    };

    std::string tostring(const Solver::MTStrategy& mtStrategy) {
        switch (mtStrategy) {
        case Solver::MTStrategy::Serial:
            return "Serial";
            break;
        case Solver::MTStrategy::MtQueryParall:
            return "Qparal";
            break;
        case Solver::MTStrategy::MtSharedTree:
            return "Shared";
            break;
        case Solver::MTStrategy::MtCopiedTrees:
            return "Copied";
            break;
        case Solver::MTStrategy::MtMultiAgent:
            return "Multiag";
            break;
        default:
            throw Error("unknown");
            break;
        }
        throw Error("unknown");
    };

    structJSON Results::getJSON() const {
        structJSON json;
        for (auto r = this->resultMatrix.begin(); r != this->resultMatrix.end(); ++r) {
            structJSON temp;
            for (auto c = r->second.begin(); c != r->second.end(); ++c) {
                temp.addElement(tostring(c->first), c->second);
            }
            json.addElement(tostring(r->first), temp);
        }
        return json;
    }

    void Results::addResult(Solver& solver, const Solver::MTStrategy& mtStrategy, const Solver::RRTStrategy& rrtStrategy, const bool& interpolateSolution) {
        structJSON result;
        result.addEndl();

        result.addElement("elapsed_time", Number<long long>(solver.getLastElapsedTime().count()));
        result.addEndl();

        result.addElement("iterations", Number<std::size_t>(solver.getLastIterations()));
        result.addEndl();

        result.addElement("threads", Number<std::size_t>(solver.getThreadAvailability()));
        result.addEndl();

        {
            arrayJSON solutionJSON;
            auto sol = solver.getLastSolution();
            if(interpolateSolution) {
                sol = interpolate(sol, *solver.getProblem().getTrajManager());
            }
            for (auto it = sol.begin(); it != sol.end(); ++it) {
                arrayJSON stateJSON;
                addValues(stateJSON, it->data(), it->size());
                solutionJSON.addElement(stateJSON);
            }
            result.addElement("solution", solutionJSON);
        }
        result.addEndl();

        {
            arrayJSON treesJSON;
            auto trees = solver.getLastTrees();
            for (auto it = trees.begin(); it != trees.end(); ++it) {
                arrayJSON treeJSON;
                auto itN = (*it)->getNodes().begin();
                arrayJSON stateJSON;
                addValues(stateJSON, (*itN)->getState().data(), (*itN)->getState().size());
                addValues(stateJSON, (*itN)->getState().data(), (*itN)->getState().size());
                treeJSON.addElement(stateJSON);
                ++itN;
                for (itN; itN != (*it)->getNodes().end(); ++itN) {
                    arrayJSON stateJSON;
                    addValues(stateJSON, (*itN)->getState().data(), (*itN)->getState().size());
                    addValues(stateJSON, (*itN)->getFather()->getState().data(), (*itN)->getFather()->getState().size());
                    treeJSON.addElement(stateJSON);
                }
                treesJSON.addElement(treeJSON);
            }
            result.addElement("trees", treesJSON);
        }
        result.addEndl();

        auto itR = this->resultMatrix.find(mtStrategy);
        if (itR == this->resultMatrix.end()) {
            itR = this->resultMatrix.emplace(mtStrategy, std::map<Solver::RRTStrategy, structJSON>{}).first;
        }
        itR->second.emplace(rrtStrategy, std::move(result));
    }

    Results::Results(Solver& solver, const NodeState& start, const NodeState& end, const std::size_t& threads) {
        auto usePossibleRrtStrategies = [&](const Solver::MTStrategy& strgt) {
            solver.solve(start, end, Solver::RRTStrategy::Single, strgt);
            this->addResult(solver, strgt, Solver::RRTStrategy::Single);

            solver.solve(start, end, Solver::RRTStrategy::Bidir, strgt);
            this->addResult(solver, strgt, Solver::RRTStrategy::Bidir);

            solver.solve(start, end, Solver::RRTStrategy::Star, strgt);
            this->addResult(solver, strgt, Solver::RRTStrategy::Star);
        };

        usePossibleRrtStrategies(Solver::MTStrategy::Serial);

        solver.setThreadAvailability(threads);

        usePossibleRrtStrategies(Solver::MTStrategy::MtQueryParall);
        usePossibleRrtStrategies(Solver::MTStrategy::MtSharedTree);
        usePossibleRrtStrategies(Solver::MTStrategy::MtCopiedTrees);

        solver.solve(start, end, Solver::RRTStrategy::Single, Solver::MTStrategy::MtMultiAgent);
        this->addResult(solver, Solver::MTStrategy::MtMultiAgent, Solver::RRTStrategy::Single);
        solver.solve(start, end, Solver::RRTStrategy::Star, Solver::MTStrategy::MtMultiAgent);
        this->addResult(solver, Solver::MTStrategy::MtMultiAgent, Solver::RRTStrategy::Star);
    }
}