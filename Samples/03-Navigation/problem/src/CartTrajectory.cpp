/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "CartTrajectory.h"
#include <complex>
#include <Checker.h>
#include <math.h>
#include <geometry/SphereLogger.h>
#include <geometry/RectangleLogger.h>
#include <PI.h>

namespace mt::traj {
    constexpr float CLOSENESS_TOLERANCE = 0.01f;

    sample::structJSON CartTrajectoryFactory::logDescription() const {
        sample::structJSON result;
        {
            sample::arrayJSON obstacles;
            for (auto it = this->description.obstacles.begin(); it != this->description.obstacles.end(); ++it) {
                obstacles.addElement(log(*it));
            }
            result.addElement("obstacles", obstacles);
        }
        result.addElement("boundaries" , log(this->description.boundaries));
        result.addElement("blending", sample::Number<float>(this->description.blendRadius));
        {
            sample::structJSON cart;
            cart.addElement("width", sample::Number<float>(this->description.cart.getWidth()));
            cart.addElement("lenght", sample::Number<float>(this->description.cart.getLength()));
            result.addElement("cart", cart);
        }
        return result;
    }

    float getSteerDegree(const sample::geometry::Rectangle& boundaries) {
        float temp1 = 0.01f * (boundaries.getXMax() - boundaries.getXMin());
        float temp2 = 0.01f * (boundaries.getYMax() - boundaries.getYMin());
        if (temp1 < temp2) return temp1;
        return temp2;
    }

    CartTrajectoryFactory::CartTrajectoryFactory(const sample::Description& description)
        : sample::SampleDescription<sample::Description>(description)
        , steerDegree(getSteerDegree(description.boundaries)) {
    }

    inline float cross(const std::complex<float>& a, const std::complex<float>& b) {
        return a.real()*b.imag() - a.imag()*b.real();
    }
    TrajectoryPtr CartTrajectoryFactory::getTrajectory(const NodeState& start, const NodeState& target) const {
        std::complex<float> startOrientation = {cosf(start[2]), sinf(start[2]) };
        std::complex<float> endOrientation = {cosf(target[2]), sinf(target[2]) };

        sample::geometry::Point startVectorBegin (start[0], start[1]);
        sample::geometry::Point startVectorEnd (start[0] + startOrientation.real(), start[1] + startOrientation.imag());
        sample::geometry::Segment startVector(startVectorBegin, startVectorEnd);

        sample::geometry::Point endVectorBegin (target[0], target[1]);
        sample::geometry::Point endVectorEnd (target[0] + endOrientation.real(), target[1] + endOrientation.imag());
        sample::geometry::Segment endVector(endVectorBegin, endVectorEnd);

        sample::geometry::LineLineChecker checker;
        checker.check(startVector, endVector);

        if(checker.wereParallel()) {
            if(checker.getDistance() > CLOSENESS_TOLERANCE) {
                return nullptr;
            }
            float dot = startOrientation.real() * (target[0] - start[0]);
            dot += startOrientation.imag() * (target[1] - start[1]);
            if((dot >= 0.0) && (fabs(start[2] - target[2]) < 0.1f)) {
                return std::make_unique<CartTrajectory>(std::make_unique<Line>(start , target, this->steerDegree) , &this->description);
            }
            return nullptr;
        }

        if(checker.getCoeffA() < 0.0) return nullptr;
        if(checker.getCoeffB() > 0.0) return nullptr;

        float coneAngleAmplitude = 0.5f * acosf(-startOrientation.real()*endOrientation.real() - startOrientation.imag()*endOrientation.imag());
        float coneAngleMiddle;
        {
            std::complex<float> middleOrientation = endOrientation - startOrientation;
            coneAngleMiddle = atan2(middleOrientation.imag(), middleOrientation.real());
        }
        float blendDistance = this->description.blendRadius / tanf(coneAngleAmplitude);
        float centerDistance2Focal = this->description.blendRadius / sinf(coneAngleAmplitude);
        
        CircleInfo info; 
        float phaseDelta;
        info.ray = this->description.blendRadius;
        bool isClockwise = false;
        if((euclideanDistance(start.data(), checker.getClosesetInA().data(), 2) < blendDistance) || 
           (euclideanDistance(target.data(), checker.getClosesetInA().data(), 2) < blendDistance)) {
            coneAngleMiddle += mt::sample::C_PI;
            phaseDelta = mt::sample::C_PI + 2.f *coneAngleAmplitude;
            startOrientation *= blendDistance;
            endOrientation *= -blendDistance;
            if((cross(startOrientation, endOrientation) < 0.f)) {
                isClockwise = true;
            }
        }
        else {
            startOrientation *= -blendDistance;
            endOrientation *= blendDistance;
            phaseDelta = 2.f * (mt::sample::C_PI_2 - coneAngleAmplitude);
            if((cross(startOrientation, endOrientation) > 0.f)) {
                isClockwise = true;
            }
        }
        phaseDelta = fabs(phaseDelta);
        info.center[0] = checker.getClosesetInA().x() + cosf(coneAngleMiddle) * centerDistance2Focal;
        info.center[1] = checker.getClosesetInA().y() + sinf(coneAngleMiddle) * centerDistance2Focal;
        NodeState blendStart = {checker.getClosesetInA().x() + startOrientation.real() , checker.getClosesetInA().y() + startOrientation.imag(), start[2]};
        NodeState blendEnd = {checker.getClosesetInA().x() + endOrientation.real()     , checker.getClosesetInA().y() + endOrientation.imag(), target[2]};
        this->lastTrajectoryCost2Go = 2.f * phaseDelta * this->description.blendRadius;
        info.phaseStart = atan2(blendStart[1] - info.center[1] , blendStart[0] - info.center[0]);
        if(isClockwise) {
            info.phaseEnd = info.phaseStart - phaseDelta;
        }
        else {
            info.phaseEnd = info.phaseStart + phaseDelta;
        }

        return std::make_unique<CartTrajectory>(std::make_unique<LineTrgSaved>(start, blendStart, this->steerDegree), 
                                                std::make_unique<Circle>(info, this->steerDegree), 
                                                std::make_unique<Line>(blendEnd, target, this->steerDegree), &this->description);
    }

    float CartTrajectoryFactory::cost2GoIgnoringConstraints(const NodeState& start, const NodeState& ending_node) const {
        auto trj = this->getTrajectory(start, ending_node);
        if(nullptr == trj) return Cost::COST_MAX;
        return this->lastTrajectoryCost2Go;
    }

    CartTrajectory::CartTrajectory(std::unique_ptr<LineTrgSaved> lineStart,std::unique_ptr<Circle> circle, std::unique_ptr<Line> lineEnd, const sample::Description* data) 
        : TrajectoryComposite(std::move(lineStart), std::move(circle), std::move(lineEnd)) {
        this->data = data;
    }

    CartTrajectory::CartTrajectory(std::unique_ptr<Line> line, const sample::Description* data) 
        : TrajectoryComposite(std::move(line)) {
        this->data = data;
    }

    AdvanceInfo CartTrajectory::advanceInternal() {
        auto info = this->TrajectoryComposite::advanceInternal();
        for (auto it = this->data->obstacles.begin(); it != this->data->obstacles.end(); ++it) {
            if (this->data->cart.isColliding((*this->piecesCursor)->getCursor().data(), *it)) {
                return traj::AdvanceInfo::blocked;
            }
        }
        return info;
    };
}
