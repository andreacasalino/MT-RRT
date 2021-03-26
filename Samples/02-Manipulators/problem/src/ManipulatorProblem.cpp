/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <ManipulatorProblem.h>
#include <sampler/HyperBox.h>
#include "Bubble.h"
#include <Error.h>
#include <list>
#include <Importer.h>
#include <PI.h>

namespace mt::sample {
    NodeState make_limit(const std::size_t& size, const float& value) {
        NodeState lim;
        lim.reserve(size);
        for (std::size_t k = 0; k < size; ++k) {
            lim.emplace_back(value);
        }
        return lim;
    }

    Description make_data(const std::vector<Manipulator>& robots, const std::vector<geometry::Sphere>& obstacles) {
        Description data;
        data.obstacles = obstacles;
        data.robots = robots;
        return data;
    }

    inline float convert(const std::string& buff) {
        return static_cast<float>(std::atof(buff.c_str()));
    }

    std::tuple<Description, NodeState, NodeState> importProblem(const std::string& configFileName) {
        Importer importer(configFileName, "robot", "start", "target");

        std::vector<Manipulator> robots;
        {
            auto robotsRaw = importer.find("robot");
            robots.reserve(robotsRaw.size());
            for(auto it =robotsRaw.begin(); it!=robotsRaw.end(); ++it) {
                robots.emplace_back(**it);
            }
        }
        std::vector<geometry::Sphere> obstacles;
        {
            auto obstaclesRaw = importer.find("obstacle");
            obstacles.reserve(obstaclesRaw.size());
            for(auto it =obstaclesRaw.begin(); it!=obstaclesRaw.end(); ++it) {
                if(3 != (*it)->size()) {
                    throw Error("invalid obstacle");
                }
                obstacles.emplace_back(geometry::Sphere((**it)[0], (**it)[1], (**it)[2]));
            }
        }
        NodeState start = *importer.find("start").front();
        if(Manipulator::dofTot(robots) != start.size()) {
            throw Error("inconsistent state size");
        }
        NodeState target = *importer.find("target").front();
        if(Manipulator::dofTot(robots) != target.size()) {
            throw Error("inconsistent state size");
        }
        return std::make_tuple(make_data(robots, obstacles), start, target);
    }

    NodeState degree2rad(const NodeState& pose) {
        NodeState converted = pose;
        for(std::size_t k=0; k<pose.size(); ++k){
            converted[k] = pose[k] * C_PI / 180.f;
        }
        return converted;
    }

    std::tuple<ProblemPtr, NodeState, NodeState> importManipulatorProblem(const std::string& configFileName) {
        auto data = importProblem(configFileName);
        std::size_t dofTot = Manipulator::dofTot(std::get<0>(data).robots);
        auto problem = std::make_unique<Problem>(std::make_unique<sampling::HyperBox>(make_limit(dofTot, -4.712389f), make_limit(dofTot, 4.712389f)),
                                         std::make_unique<traj::BubbleFactory>(std::get<0>(data)),
                                         dofTot, 10.f);
        return std::make_tuple(std::move(problem), std::get<1>(data),  std::get<2>(data));
    }
}