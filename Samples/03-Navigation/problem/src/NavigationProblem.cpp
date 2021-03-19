/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <NavigationProblem.h>
#include <geometry/SphereLogger.h>
#include <geometry/RectangleLogger.h>

namespace mt::sample {



    structJSON NavigationProblem::getJSON() const {
        structJSON result;
        {
            arrayJSON obstacles;
            for (auto it = this->getObstacles().begin(); it != this->getObstacles().end(); ++it) {
                obstacles.addElement(log(*it));
            }
            result.addElement("obstacles", obstacles);
        }
        result.addElement("boundaries" , log(this->getBoundaries()));
        throw Error("log cart info");
        return result;
    }
}
