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

    Logger::Logger(mt::Solver& solver) {
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
        std::ofstream f(fileName);
        if (!f.is_open()) {
            throw Error("invalid log file");
        }
        f << this->data.str();
    }
}