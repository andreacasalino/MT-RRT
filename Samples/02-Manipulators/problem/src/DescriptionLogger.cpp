/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "DescriptionLogger.h"
#include <geometry/SphereLogger.h>

namespace mt::sample {
    sample::structJSON DescriptionLogger::logDescription() const {
        sample::structJSON result;
        {
            sample::arrayJSON obstacles;
            for (auto it = this->description.obstacles.begin(); it != this->description.obstacles.end(); ++it) {
                obstacles.addElement(log(*it));
            }
            result.addElement("obstacles", obstacles);
        }
        {
            sample::arrayJSON robots;
            for (auto it = this->description.robots.begin(); it != this->description.robots.end(); ++it) {
                sample::arrayJSON temp;
                temp.addElement(sample::Number<float>(it->getBase().x()));
                temp.addElement(sample::Number<float>(it->getBase().y()));
                temp.addElement(sample::Number<float>(it->getBase().z()));
                for (auto l = it->getLinks().begin(); l != it->getLinks().end(); ++l) {
                    temp.addElement(sample::Number<float>(l->length.get()));
                }
                for (auto l = it->getLinks().begin(); l != it->getLinks().end(); ++l) {
                    temp.addElement(sample::Number<float>(l->ray.get()));
                }
                robots.addElement(temp);
            }
            result.addElement("robots", robots);

        }
        return result;
    }

    DescriptionLogger::DescriptionLogger(const sample::Description& description)
        : sample::SampleDescription<sample::Description>(description) {
    }
}