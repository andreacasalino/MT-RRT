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
#include <iostream>

#include <strategies/SerialStrategy.h>
#include <strategies/QueryParallStrategy.h>
#include <strategies/SharedTreeStrategy.h>
#include <strategies/LinkedTreesStrategy.h>
#include <strategies/MultiAgentStrategy.h>

namespace mt::sample {
    arrayJSON makeValues(const NodeState& state) {
        arrayJSON temp;
        addValues(temp, state.data(), state.size());
        return temp;
    };

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

    std::string tostring(const solver::RRTStrategy& rrtStrategy) {
        switch (rrtStrategy) {
        case solver::RRTStrategy::Single:
            return "Single";
        case solver::RRTStrategy::Bidir:
            return "Bidir";
        case solver::RRTStrategy::Star:
            return "Star";
        default:
            throw Error("unknown");
            break;
        }
        throw Error("unknown");
    };

    std::string tostring(const StrategyType& mtStrategy) {
        switch (mtStrategy) {
        case StrategyType::Serial:
            return "Serial";
        case StrategyType::MtQueryParall:
            return "Qparal";
        case StrategyType::MtSharedTree:
            return "Shared";
        case StrategyType::MtLinkedTrees:
            return "Copied";
        case StrategyType::MtMultiAgent:
            return "Multiag";
        default:
            throw Error("unknown");
            break;
        }
        throw Error("unknown");
    };

    std::unique_ptr<solver::Strategy> make_strategy(const StrategyType& type) {
        switch (type) {
        case StrategyType::Serial:
            return std::make_unique<solver::SerialStrategy>();
        case StrategyType::MtQueryParall:
            return std::make_unique<solver::QueryParallStrategy>();
        case StrategyType::MtSharedTree:
            return std::make_unique<solver::SharedTreeStrategy>();
        case StrategyType::MtLinkedTrees:
            return std::make_unique<solver::LinkedTreesStrategy>();
        case StrategyType::MtMultiAgent:
            return std::make_unique<solver::MultiAgentStrategy>();
        default:
            throw Error("unknown");
            break;
        }
        return nullptr;
    }

    void setStrategy(solver::Solver& solver,  const StrategyType& type, const StrategyParameter& parameters) {
        auto strategy = make_strategy(type);
        strategy->getIterationsMax().set(parameters.iterations);
        solver.setSteerTrials(parameters.steerTrials);
        strategy->getDeterministicCoefficient().set(parameters.determinism);
        solver.setThreadAvailability(parameters.threads);
        solver.saveTreesAfterSolve();
        solver.setStrategy(std::move(strategy));
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

    void Results::addResult(solver::Solver& solver, const StrategyType& mtStrategy, const solver::RRTStrategy& rrtStrategy) {
        structJSON result;
        result.addEndl();

        result.addElement("elapsed_time", Number<long long>(solver.getLastElapsedTime().count()));
        result.addEndl();

        result.addElement("iterations", Number<std::size_t>(solver.getLastIterations()));
        result.addEndl();

        result.addElement("threads", Number<std::size_t>(solver.getThreadAvailability()));
        result.addEndl();

        result.addElement("start", makeValues(solver.getLastStart()));
        result.addEndl();

        result.addElement("target", makeValues(solver.getLastTarget()));
        result.addEndl();

        {
            arrayJSON solutionJSON;
            auto sol = solver.copyLastSolution();
            // interpolate the solution
            auto interp = [&sol](const Problem& p){
                sol = interpolate(sol, *p.getTrajManager());
            };
            if (nullptr == this->interpolator) {
                solver.useProblem(interp);
            }
            else {
                interp(*this->interpolator);
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
            auto trees = solver.extractLastTrees();
            for (auto it = trees.begin(); it != trees.end(); ++it) {
                arrayJSON treeJSON;
                auto itNend = (*it)->rend();
                --itNend;
                auto itN = (*it)->rbegin();
                for(itN; itN!=itNend; ++itN) {
                    arrayJSON stateJSON;
                    addValues(stateJSON, (*itN)->getState().data(), (*itN)->getState().size());
                    addValues(stateJSON, (*itN)->getFather()->getState().data(), (*itN)->getFather()->getState().size());
                    treeJSON.addElement(stateJSON);
                }
                arrayJSON stateJSON;
                addValues(stateJSON, (*itN)->getState().data(), (*itN)->getState().size());
                addValues(stateJSON, (*itN)->getState().data(), (*itN)->getState().size());
                treeJSON.addElement(stateJSON);
                treesJSON.addElement(treeJSON);
            }
            result.addElement("trees", treesJSON);
        }
        result.addEndl();

        auto itR = this->resultMatrix.find(mtStrategy);
        if (itR == this->resultMatrix.end()) {
            itR = this->resultMatrix.emplace(mtStrategy, std::map<solver::RRTStrategy, structJSON>{}).first;
        }
        itR->second.emplace(rrtStrategy, std::move(result));
    }

    Results::Results(ProblemPtr interpolator) {
        this->interpolator = std::move(interpolator);
    }

    Results::Results(solver::Solver& solver, const NodeState& start, const NodeState& end, const StrategyParameter& parameters, ProblemPtr interpolator)
        : Results(std::move(interpolator)) {
        auto usePossibleRrtStrategies = [&](const StrategyType& strgt) {
            setStrategy(solver, strgt, parameters);

            solver.solve(start, end, solver::RRTStrategy::Single);
            this->addResult(solver, strgt, solver::RRTStrategy::Single);

            try {
                solver.solve(start, end, solver::RRTStrategy::Bidir);
                this->addResult(solver, strgt, solver::RRTStrategy::Bidir);
            }
            catch(...) {
            }
            
            solver.solve(start, end, solver::RRTStrategy::Star);
            this->addResult(solver, strgt, solver::RRTStrategy::Star);
        };

        std::cout << "Serial started" << std::endl;
        usePossibleRrtStrategies(StrategyType::Serial);
        std::cout << "done" << std::endl;

        std::cout << "Query Parall started" << std::endl;
        usePossibleRrtStrategies(StrategyType::MtQueryParall);
        std::cout << "done" << std::endl;

        std::cout << "Shared tree started" << std::endl;
        usePossibleRrtStrategies(StrategyType::MtSharedTree);
        std::cout << "done" << std::endl;

        std::cout << "Copied trees started" << std::endl;
        usePossibleRrtStrategies(StrategyType::MtLinkedTrees);
        std::cout << "done" << std::endl;

        std::cout << "Multi agent started" << std::endl;
        usePossibleRrtStrategies(StrategyType::MtMultiAgent);
        std::cout << "done" << std::endl;
    }
}