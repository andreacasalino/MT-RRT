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

        if(Manipulator::dofTot(robots) != start.size()) {
            throw Error("inconsistent state size");
        }
        if(Manipulator::dofTot(robots) != target.size()) {
            throw Error("inconsistent state size");
        }
        return std::make_tuple(make_data(robots, obstacles), start, target);
    }

    NodeState degree2rad(const NodeState& pose) {
        NodeState converted = pose;
        for(std::size_t k=0; k<pose.size(); ++k){
            converted[k] = pose[k] * 3.141f / 180.f;
        }
        return converted;
    }

    std::tuple<ProblemPtr, NodeState, NodeState> importManipulatorProblem(const std::string& configFileName) {
        auto data = importProblem(configFileName);
        std::size_t dofTot = Manipulator::dofTot(std::get<0>(data).robots);
        auto problem = std::make_unique<Problem>(std::make_unique<sampling::HyperBox>(make_limit(dofTot, -4.712389f), make_limit(dofTot, 4.712389f)),
                                         std::make_unique<traj::BubbleFactory>(std::get<0>(data)),
                                         2, 500.f);
        return std::make_tuple(std::move(problem), std::get<1>(data),  std::get<2>(data));
    }
}