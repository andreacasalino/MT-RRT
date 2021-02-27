/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Point2D.h>
#include <sampler/Box.h>
#include "Line.h"

namespace mt::sample {
    float getSteerDegree(const sample::Box& boundaries) {
        float temp1 = 0.05f * (boundaries.getXMax() - boundaries.getXMin());
        float temp2 = 0.05f * (boundaries.getXMax() - boundaries.getXMin());
        if (temp1 < temp2) return temp1;
        return temp2;
    }

    sampling::SamplerPtr getSampler(const sample::Box& boundaries) {
        NodeState low = { boundaries.getXMin() , boundaries.getXMin() };
        NodeState upp = { boundaries.getXMax() , boundaries.getYMax() };
        return std::make_unique<sampling::Box>(low, upp);
    }

    Point2D::Point2D(const sample::Box& boundaries, const std::vector<sample::Box>& obstacles)
        : Problem(getSampler(boundaries), 
                  std::make_unique<traj::LineManager>(getSteerDegree(boundaries), obstacles), 
                  2, 10.f) {
    }

    std::vector<sample::Box> Point2D::getObstacles() const {
        return static_cast<traj::LineManager*>(this->trajManager.get())->getObstacles();
    }

    sample::Box Point2D::getBoundaries() const {
        const sampling::Box* ptSam = static_cast<const sampling::Box*>(this->sampler.get());
        geometry::Point A(ptSam->getLowerLimit().front(), ptSam->getLowerLimit().back());
        geometry::Point B(ptSam->getLowerLimit().front() + ptSam->getDeltaLimit().front(), ptSam->getLowerLimit().back() + ptSam->getDeltaLimit().back());
        return sample::Box(A, B);
    }

    void Point2D::log(Logger& log) const {
        auto obst = this->getObstacles();
        arrayJSON obstJSON;
        for (auto it = obst.begin(); it != obst.end(); ++it) {
            arrayJSON temp;
            temp.addElement(Number<float>(it->getXMin()));
            temp.addElement(Number<float>(it->getYMin()));
            temp.addElement(Number<float>(it->getXMax()));
            temp.addElement(Number<float>(it->getYMax()));
            obstJSON.addElement(temp);
        }
        log.addElement("boxes" , obstJSON);
        log.addEndl();
    }
}
