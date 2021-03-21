/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <NavigationProblem.h>
#include <Logger.h>
using namespace std;

mt::sample::arrayJSON convert(const mt::NodeState& state) {
    mt::sample::arrayJSON result;
    for(std::size_t k=0; k<state.size(); ++k) {
        result.addElement(mt::sample::Number<float>(state[k]));
    }
    return result;
};

int main() {
    mt::ProblemPtr problem;
    {
        auto data = mt::sample::importNavigationProblem(std::string(CONFIG_FOLDER) + "/Sample00-config");
        problem = std::move(std::get<0>(data));
    }

    mt::sample::arrayJSON trajGenLog;
    auto addTraj = [&trajGenLog, &problem](const mt::NodeState& start, const mt::NodeState& end){
        mt::sample::structJSON trjLog;
        trjLog.addElement("start", convert(start));
        trjLog.addElement("end", convert(end));
        mt::sample::arrayJSON statesLog;
        {
            auto trj = problem->getTrajManager()->getTrajectory(start, end);
            if(nullptr != trj) {
                do {
                    statesLog.addElement(convert(trj->getCursor()));
                } while (trj->advance() == mt::traj::AdvanceInfo::advanced);
                statesLog.addElement(convert(trj->getCursor()));
            }
        }
        trjLog.addElement("states", statesLog);
        trajGenLog.addElement(trjLog);
    };

    addTraj(std::vector<float>{0.f, 0.f, 0.f}, std::vector<float>{100.f, 100.f, 0.5f * 3.141f});
    addTraj(std::vector<float>{0.f, 0.f, 0.f}, std::vector<float>{100.f, 100.f, 3.f * 3.141f / 4.f});
    addTraj(std::vector<float>{80.f, 0.f, 0.f}, std::vector<float>{100.f, 100.f, 0.5f * 3.141f});
    addTraj(std::vector<float>{0.f, 0.f, 0.f}, std::vector<float>{200.f, 100.f, 0.25f * 3.141f});
    addTraj(std::vector<float>{0.f, 0.f, -0.25f * 3.141f}, std::vector<float>{200.f, 100.f, 0.25f * 3.141f});
    for(std::size_t k=0; k<20; ++k) {
        addTraj(problem->getSampler()->randomState() , problem->getSampler()->randomState());
    }

    mt::sample::printData(trajGenLog , "TrajGen.json");

    return EXIT_SUCCESS;
}