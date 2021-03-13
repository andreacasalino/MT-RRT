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
}