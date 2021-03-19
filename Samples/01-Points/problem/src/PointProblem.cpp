/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <PointProblem.h>
#include <sampler/HyperBox.h>
#include "LineWithCheck.h"
#include <geometry/RectangleLogger.h>

namespace mt::sample {
    float getSteerDegree(const geometry::Rectangle& boundaries) {
        float temp1 = 0.05f * (boundaries.getXMax() - boundaries.getXMin());
        float temp2 = 0.05f * (boundaries.getYMax() - boundaries.getYMin());
        if (temp1 < temp2) return temp1;
        return temp2;
    }

    sampling::SamplerPtr make_sampler(const geometry::Rectangle& boundaries) {
        NodeState low = { boundaries.getXMin() , boundaries.getYMin() };
        NodeState upp = { boundaries.getXMax() , boundaries.getYMax() };
        return std::make_unique<sampling::HyperBox>(low, upp);
    }

    PointProblem::PointProblem(const geometry::Rectangle& boundaries, const std::vector<geometry::Rectangle>& obstacles) 
        : Problem(make_sampler(boundaries),
                  std::make_unique<traj::LineWithCheckFactory>(getSteerDegree(boundaries), obstacles),
                  2, 500.f) {
    }

    const std::vector<geometry::Rectangle>& PointProblem::getObstacles() const {
        return static_cast<traj::LineWithCheckFactory*>(this->trajManager.get())->getObstacles();
    }

    geometry::Rectangle PointProblem::getBoundaries() const {
        const sampling::HyperBox* ptSam = static_cast<const sampling::HyperBox*>(this->sampler.get());
        geometry::Point A(ptSam->getLowerLimit().front(), ptSam->getLowerLimit().back());
        geometry::Point B(ptSam->getLowerLimit().front() + ptSam->getDeltaLimit().front(), ptSam->getLowerLimit().back() + ptSam->getDeltaLimit().back());
        return geometry::Rectangle(A, B);
    }

    structJSON PointProblem::getJSON() const {
        structJSON result;

        result.addElement("limits", log(this->getBoundaries()));

        arrayJSON obstacles;
        const std::vector<geometry::Rectangle>& obst = this->getObstacles();
        for (auto it = obst.begin(); it != obst.end(); ++it) {
            obstacles.addElement(log(*it));
        }
        result.addElement("obstacles", obstacles);

        return result;
    }
}
