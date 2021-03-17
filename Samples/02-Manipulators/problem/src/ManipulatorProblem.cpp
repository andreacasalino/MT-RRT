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
#include <fstream>
#include <list>

namespace mt::sample {
    NodeState make_limit(const std::size_t& size, const float& value) {
        NodeState lim;
        lim.reserve(size);
        for (std::size_t k = 0; k < size; ++k) {
            lim.emplace_back(value);
        }
        return lim;
    }

    sample::ProblemData make_data(const std::vector<Manipulator>& robots, const std::vector<geometry::Sphere>& obstacles) {
        sample::ProblemData data;
        data.obstacles = obstacles;
        data.robots = robots;
        return data;
    }

    ManipulatorProblem::ManipulatorProblem(const std::vector<Manipulator>& robots, const std::vector<geometry::Sphere>& obstacles)
        : Problem(std::make_unique<sampling::HyperBox>(make_limit(Manipulator::dofTot(robots), -4.712389f), make_limit(Manipulator::dofTot(robots), 4.712389f)),
            std::make_unique<traj::BubbleManager>(2 * 3.141f / 180.f, make_data(robots, obstacles)),
            Manipulator::dofTot(robots), 5.f) {
    }

    const std::vector<Manipulator>& ManipulatorProblem::getRobots() const {
        return static_cast<traj::BubbleManager&>(*this->trajManager).getData().robots;
    };

    const std::vector<geometry::Sphere>& ManipulatorProblem::getObstacles() const {
        return static_cast<traj::BubbleManager&>(*this->trajManager).getData().obstacles;
    };

    structJSON ManipulatorProblem::getJSON() const {
        structJSON result;
        {
            arrayJSON obstacles;
            for (auto it = this->getObstacles().begin(); it != this->getObstacles().end(); ++it) {
                arrayJSON temp;
                temp.addElement(Number<float>(it->getCenter().x()));
                temp.addElement(Number<float>(it->getCenter().y()));
                temp.addElement(Number<float>(it->getRay()));
                obstacles.addElement(temp);
            }
            result.addElement("obstacles", obstacles);
        }
        {
            arrayJSON robots;
            for (auto it = this->getRobots().begin(); it != this->getRobots().end(); ++it) {
                arrayJSON temp;
                temp.addElement(Number<float>(it->getBase().x()));
                temp.addElement(Number<float>(it->getBase().y()));
                for (auto l = it->getLinks().begin(); l != it->getLinks().end(); ++l) {
                    temp.addElement(Number<float>(l->length.get()));
                }
                for (auto l = it->getLinks().begin(); l != it->getLinks().end(); ++l) {
                    temp.addElement(Number<float>(l->ray.get()));
                }
                robots.addElement(temp);
            }
            result.addElement("robots", robots);

        }
        return result;
    }

    inline float convert(const std::string& buff) {
        return static_cast<float>(std::atof(buff.c_str()));
    }

    std::tuple<ProblemPtr, NodeState, NodeState> importProblem(const std::string& configFileName) {
        std::ifstream f(configFileName);
        if(!f.is_open()) {
            throw Error("invalid config file");
        }

        auto trimmer = [](const std::string& line) -> std::list<std::string> {
            std::istringstream iss(line);
            std::list<std::string> slices;
            while (!iss.eof()) {
                slices.emplace_back(std::string());
                iss >> slices.back();
                if(slices.back().empty()) slices.pop_back();
            }
            return slices;
        };

        std::vector<Manipulator> robots;
        std::vector<geometry::Sphere> obstacles;
        NodeState start;
        NodeState target;

        std::string line;
        std::list<std::string> slices;
        while (!f.eof()) {
            std::getline(f, line);
            slices = trimmer(line);

            if(!slices.empty()) {
                if(0 == slices.front().compare("obstacle")) {
                    slices.pop_front();
                    float coor[3];
                    coor[0] = convert(slices.front());
                    slices.pop_front();
                    coor[1] = convert(slices.front());
                    slices.pop_front();
                    coor[2] = convert(slices.front());
                    slices.pop_front();
                    obstacles.push_back(geometry::Sphere(coor[0], coor[1], coor[2]));
                }
                else if(0 == slices.front().compare("robot")) {
                    slices.pop_front();
                    if(slices.size() % 2 != 0) {
                        throw Error("invalid robot data");
                    }
                    if(slices.size() <= 2) {
                        throw Error("invalid robot data");
                    }
                    float bx = convert(slices.front());
                    slices.pop_front();
                    float by = convert(slices.front());
                    slices.pop_front();
                    geometry::Point base(bx, by, 0.f);
                    std::vector<Manipulator::Link> links;
                    while (!slices.empty()) {
                        Manipulator::Link temp;
                        temp.length = convert(slices.front());
                        slices.pop_front();
                        temp.ray = convert(slices.front());
                        slices.pop_front();
                        links.push_back(temp);
                    }
                    robots.push_back(Manipulator(base , links));
                }
                else if(0 == slices.front().compare("start")) {
                    slices.pop_front();
                    if(!start.empty()) {
                        throw Error("multiple start configurations found");
                    }
                    while (!slices.empty()) {
                        start.push_back(convert(slices.front()));
                        slices.pop_front();
                    }
                }
                else if(0 == slices.front().compare("target")) {
                    slices.pop_front();
                    if(!target.empty()) {
                        throw Error("multiple target configurations found");
                    }
                    while (!slices.empty()) {
                        target.push_back(convert(slices.front()));
                        slices.pop_front();
                    }
                }
            }
        }

        ProblemPtr problem = std::make_unique<ManipulatorProblem>(robots, obstacles);
        if(problem->getProblemSize() != start.size()) {
            throw Error("inconsistent state size");
        }
        if(problem->getProblemSize() != target.size()) {
            throw Error("inconsistent state size");
        }
        return std::make_tuple(std::move(problem), start, target);
    }

    NodeState degree2rad(const NodeState& pose) {
        NodeState converted = pose;
        for(std::size_t k=0; k<pose.size(); ++k){
            converted[k] = pose[k] * 3.141f / 180.f;
        }
        return converted;
    }
}