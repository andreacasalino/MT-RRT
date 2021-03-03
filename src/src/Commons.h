/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_COMMONS_H
#define MT_RRT_COMMONS_H

#include <Problem.h>
#include <math.h>

namespace mt {
    class MtObject {
    public:
        MtObject(const std::size_t& threadsNumber) : threadsNumber(threadsNumber) {};

        inline std::size_t getThreadsNumber() const { return this->threadsNumber; };

    private:
        const std::size_t threadsNumber;
    };

    class ProblemBattery : public MtObject {
    public:
        ProblemBattery(const std::vector<ProblemPtr>& problemcopies);
        ProblemBattery(const ProblemBattery& o) = default;

    protected:
        inline Problem& getProblem(const std::size_t& pos) { return *this->problems[pos]; };
        inline const Problem& getProblem(const std::size_t& pos) const { return *this->problems[pos]; };

        std::vector<Problem*> problems;
    };

    inline std::size_t computeBatchSize(const std::size_t& iterations, const double& reallCoeff, const std::size_t& threads) {
        return static_cast<size_t>(ceil(reallCoeff* static_cast<double>(iterations) / static_cast<double>(threads)));
    }
}

#endif