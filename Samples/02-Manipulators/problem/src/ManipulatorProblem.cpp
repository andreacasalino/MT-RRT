/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <ManipulatorProblem.h>
#include <sampler/HyperBox.h>
#include "Line.h"
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

    std::shared_ptr<sample::ProblemData> make_data(const std::vector<Manipulator>& robots, const std::vector<Sphere>& obstacles) {
        std::shared_ptr<sample::ProblemData> data = std::make_shared<sample::ProblemData>();
        data->obstacles = obstacles;
        data->robots = robots;
        return data;
    }

    ManipulatorProblem::ManipulatorProblem(const std::vector<Manipulator>& robots, const std::vector<Sphere>& obstacles)
        : Problem(std::make_unique<sampling::HyperBox>(make_limit(Manipulator::dofTot(robots), -4.712389f), make_limit(Manipulator::dofTot(robots), 4.712389f)),
            std::make_unique<traj::LineManager>(2 * 3.141f / 180.f, make_data(robots, obstacles)),
            Manipulator::dofTot(robots), 5.f) {
        this->data = static_cast<traj::LineManager&>(*this->trajManager).getData();
    }

    structJSON ManipulatorProblem::getJSON() const {
        structJSON result;
        {
            arrayJSON obstacles;
            for (auto it = this->data->obstacles.begin(); it != this->data->obstacles.end(); ++it) {
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
            for (auto it = this->data->robots.begin(); it != this->data->robots.end(); ++it) {
                arrayJSON temp;
                temp.addElement(Number<float>(it->getBase().x()));
                temp.addElement(Number<float>(it->getBase().y()));
                for (auto l = it->getLinks().begin(); l != it->getLinks().end(); ++l) {
                    temp.addElement(Number<float>(l->length));
                }
                for (auto l = it->getLinks().begin(); l != it->getLinks().end(); ++l) {
                    temp.addElement(Number<float>(l->ray));
                }
                robots.addElement(temp);
            }
            result.addElement("robots", robots);

        }
        return result;
    }

    inline float convert(const std::string& buff) {
        return static_cast<float>(std::atoi(buff.c_str()));
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
        std::vector<Sphere> obstacles;
        NodeState start;
        NodeState target;

        std::string line;
        std::list<std::string> slices;
        while (!f.eof()) {
            std::getline(f, line);
            slices = trimmer(line);

            if(slices.empty()) {
                throw Error("invalid config file");
            }

            if(0 == slices.front().compare("obstacle")) {
                slices.pop_back();
                float coor[3];
                coor[0] = convert(slices.front());
                slices.pop_back();
                coor[1] = convert(slices.front());
                slices.pop_back();
                coor[2] = convert(slices.front());
                slices.pop_back();
                obstacles.push_back(Sphere(coor[0], coor[1], coor[2]));
            }
            else if(0 == slices.front().compare("robot")) {
                slices.pop_back();
                if(slices.size() % 2 != 0) {
                    throw Error("invalid robot data");
                }
                if(slices.size() <= 2) {
                    throw Error("invalid robot data");
                }
                float bx = convert(slices.front());
                slices.pop_back();
                float by = convert(slices.front());
                slices.pop_back();
                geometry::Point base(bx, by, 0.f);
                std::vector<Manipulator::Link> links;
                while (!slices.empty()) {
                    Manipulator::Link temp;
                    temp.length = convert(slices.front());
                    slices.front();
                    temp.ray = convert(slices.front());
                    slices.front();
                    links.push_back(temp);
                }
                robots.push_back(Manipulator(base , links));
            }
            else if(0 == slices.front().compare("start")) {
                slices.pop_back();
                if(!start.empty()) {
                    throw Error("multiple start configurations found");
                }
                while (!slices.empty()) {
                    start.push_back(convert(slices.front()));
                }
            }
            else if(0 == slices.front().compare("target")) {
                slices.pop_back();
                if(!target.empty()) {
                    throw Error("multiple target configurations found");
                }
                while (!slices.empty()) {
                    target.push_back(convert(slices.front()));
                }
            }
            else{
                throw Error("invalid config file");
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
}