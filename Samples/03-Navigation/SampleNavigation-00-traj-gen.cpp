/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <NavigationProblem.h>
#include <SampleDescription.h>
#include <Logger.h>
#include <math.h>
#include <PI.h>
using namespace std;


class DebugLog {
public:
    DebugLog() {
        auto data = mt::sample::importNavigationProblem(std::string(CONFIG_FOLDER) + "/Sample00-config");
        this->problem = std::move(std::get<0>(data));
	    this->problemLog = dynamic_cast<const mt::sample::SampleDescription<mt::sample::Description>*>(problem->getTrajManager())->logDescription();
    };
    ~DebugLog() {
        mt::sample::structJSON log;
        log.addElement("problem", this->problemLog);
        log.addElement("traj", this->trajLog);
        mt::sample::printData(log , "DebugLog.json");
    };

    inline mt::sampling::Sampler* getSampler() const { return this->problem->getSampler(); };

    void addTrajectory(const mt::NodeState& start, const mt::NodeState& end) {
        mt::sample::structJSON trjLogNew;
        trjLogNew.addElement("start", convert(start));
        trjLogNew.addElement("end", convert(end));
        mt::sample::arrayJSON statesLog;
        auto trj = problem->getTrajManager()->getTrajectory(start, end);
        if(nullptr != trj) {
            do {
                statesLog.addElement(convert(trj->getCursor(), trj->getCumulatedCost()));
            } while (trj->advance() == mt::traj::AdvanceInfo::advanced);
            statesLog.addElement(convert(trj->getCursor(), trj->getCumulatedCost()));
        }
        trjLogNew.addElement("states", statesLog);
        this->trajLog.addElement(trjLogNew);
    };

private:
    static mt::sample::arrayJSON convert(const mt::NodeState& state) {
        mt::sample::arrayJSON result;
        for(std::size_t k=0; k<state.size(); ++k) {
            result.addElement(mt::sample::Number<float>(state[k]));
        }
        return result;
    };
    static mt::sample::arrayJSON convert(const mt::NodeState& state, const float& cost) {
        mt::sample::arrayJSON result = convert(state);
        result.addElement(mt::sample::Number<float>(cost));
        return result;
    };

    mt::ProblemPtr problem;
    mt::sample::structJSON problemLog;
    mt::sample::arrayJSON trajLog;
};

int main() {
    DebugLog logger;

    logger.addTrajectory(std::vector<float>{0.f, 0.f, 0.25f * mt::sample::C_PI}, std::vector<float>{100.f, 100.f, 0.25f * mt::sample::C_PI});
    logger.addTrajectory(std::vector<float>{0.f, 0.f, 0.25f * mt::sample::C_PI}, std::vector<float>{100.f, 100.f, 0.f});
    logger.addTrajectory(std::vector<float>{0.f, 0.f, 0.f}, std::vector<float>{100.f, 100.f, 0.5f * mt::sample::C_PI});
    logger.addTrajectory(std::vector<float>{0.f, 0.f, 0.f}, std::vector<float>{100.f, 100.f, 3.f * mt::sample::C_PI / 4.f});
    logger.addTrajectory(std::vector<float>{0.f, 0.f, 0.f}, std::vector<float>{-100.f, -100.f, -3.f * mt::sample::C_PI / 4.f});
    logger.addTrajectory(std::vector<float>{0.f, 0.f, 0.f}, std::vector<float>{100.f, -100.f, -3.f * mt::sample::C_PI / 4.f});
    logger.addTrajectory(std::vector<float>{80.f, 0.f, 0.f}, std::vector<float>{100.f, 100.f, 0.5f * mt::sample::C_PI});
    logger.addTrajectory(std::vector<float>{0.f, 0.f, 0.f}, std::vector<float>{200.f, 100.f, 0.25f * mt::sample::C_PI});
    logger.addTrajectory(std::vector<float>{0.f, 0.f, -0.25f * mt::sample::C_PI}, std::vector<float>{200.f, 100.f, 0.25f * mt::sample::C_PI});
    for(std::size_t k=0; k<5; ++k) {
        logger.addTrajectory(logger.getSampler()->randomState() , logger.getSampler()->randomState());
    }

    return EXIT_SUCCESS;
}