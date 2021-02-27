/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Logger.h>
#include <fstream>
#include <Error.h>

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

    Logger::Logger(mt::Solver& solver) {
        this->data.addEndl();

        this->data.addElement("elapsed_time", Number<long long>(solver.getLastElapsedTime().count()));
        this->data.addEndl();

        this->data.addElement("iterations" , Number<std::size_t>(solver.getLastIterations()) );
        this->data.addEndl();

        {
            arrayJSON solutionJSON;
            auto sol = solver.getLastSolution();
            for (auto it = sol.begin(); it != sol.end(); ++it) {
                arrayJSON stateJSON;
                addValues(stateJSON, it->data(), it->size());
                solutionJSON.addElement(stateJSON);
            }
            this->data.addElement("solution", solutionJSON);
        }
        this->data.addEndl();
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
            this->data.addElement("trees", treesJSON);
        }
        this->data.addEndl();
    }

    void Logger::print(const std::string& fileName) {
        printData(this->data, fileName);
    }

    std::unique_ptr<structJSON> Logger::logStrategies(mt::Solver& solver, const mt::NodeState& start, const mt::NodeState& end, const std::size_t& threads) {
        arrayJSON results;
        auto solveAndLog = [&](const mt::Solver::MTStrategy& strategy, const std::string& strategyLogName) {
            arrayJSON res;
            solver.solve( start, end, Solver::RRTStrategy::Single, strategy);
            res.addElement(Logger(solver).data);
            if (solver.getProblem().isProblemSimmetric()) {
                solver.solve(start, end, Solver::RRTStrategy::Bidir, strategy);
                res.addElement(Logger(solver).data);
            }
            solver.solve(start, end, Solver::RRTStrategy::Star, strategy);
            res.addElement(Logger(solver).data);
            structJSON temp;
            temp.addElement(strategyLogName, res);
            temp.addElement("threads", Number<std::size_t>(solver.getThreadAvailability()));
            results.addElement(temp);
        };

        solver.setThreadAvailability(1);
        solveAndLog(mt::Solver::MTStrategy::Serial, "serial");

        solver.setThreadAvailability(threads);
        solveAndLog(mt::Solver::MTStrategy::MtQueryParall, "quary parall");
        solveAndLog(mt::Solver::MTStrategy::MtSharedTree, "shared tree");
        solveAndLog(mt::Solver::MTStrategy::MtCopiedTrees, "copied trees");
        solveAndLog(mt::Solver::MTStrategy::MtMultiAgent, "multiagent");

        std::unique_ptr<structJSON> temp;
        temp->addElement("results", results);
        return temp;
    }
}