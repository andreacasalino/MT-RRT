/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <ProblemPoint.h>

namespace mt::sample {
    class BoxesChecker : public Checker {
    public:
        BoxesChecker(const std::vector<Box>& obstacles) : obstacles(obstacles) {};

        inline std::unique_ptr<Checker> copy() const override { return std::make_unique<BoxesChecker>(*this); };

        inline bool isNotAdmitted(const NodeState& state) override;

    private:
        const std::vector<Box> obstacles;
    };


}
