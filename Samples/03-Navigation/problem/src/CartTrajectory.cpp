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

        float angleMiddle;
        {
            std::complex<float> middleDir = endOrientation;
            middleDir -= startOrientation;
            angleMiddle = atan2(middleDir.imag() , middleDir.real());
        }

        float angleDelta = fabs(angleMiddle - target[2]);
        float blendDistance = this->description.blendRadius / tanf(angleDelta );
        
        bool useLongerArc = false;
        if((euclideanDistance(start.data(), checker.getClosesetInA().data(), 2) < blendDistance) || 
           (euclideanDistance(target.data(), checker.getClosesetInA().data(), 2) < blendDistance)) {
            angleMiddle += M_PI;
            useLongerArc = true;
            startOrientation *= blendDistance;
            endOrientation *= -blendDistance;
        }
        else {
            startOrientation *= -blendDistance;
            endOrientation *= blendDistance;
        }
        this->blendStart = {checker.getClosesetInA().x() + startOrientation.real() , checker.getClosesetInA().y() + startOrientation.imag(), start[2]};
        this->blendEnd = {checker.getClosesetInA().x() + endOrientation.real()     , checker.getClosesetInA().y() + endOrientation.imag(), target[2]};

        float centerDistance2Focal = this->description.blendRadius / sinf(angleDelta);
        this->blendInfo.centerX = checker.getClosesetInA().x() + cosf(angleMiddle) * centerDistance2Focal;
        this->blendInfo.centerY = checker.getClosesetInA().y() + sinf(angleMiddle) * centerDistance2Focal;

        this->blendInfo.angleStart = atan2(this->blendStart[1] - this->blendInfo.centerY, this->blendStart[0] - this->blendInfo.centerX);
        this->blendInfo.angleEnd = atan2(this->blendEnd[1] - this->blendInfo.centerY, this->blendEnd[0] - this->blendInfo.centerX);
        if(useLongerArc) {
            if(fabs(this->blendInfo.angleEnd - this->blendInfo.angleStart) < M_PI) {
                if(this->blendInfo.angleEnd > 0.f) this->blendInfo.angleEnd -= 2.f * M_PI;
                else                               this->blendInfo.angleEnd += 2.f * M_PI;
            }
        }
        this->blendInfo.ray = this->description.blendRadius;

        return std::make_unique<CartTrajectory>(std::make_unique<LineTrgSaved>(start, blendStart, this->steerDegree), 
                                                std::make_unique<Circle>(blendInfo, this->steerDegree), 
                                                std::make_unique<Line>(blendEnd, target, this->steerDegree), &this->description);
    }

    float CartTrajectoryFactory::cost2GoIgnoringConstraints(const NodeState& start, const NodeState& ending_node) const {
        auto trj = this->getTrajectory(start, ending_node);
        if(nullptr == trj) return Cost::COST_MAX;
        float cost = euclideanDistance(start.data(), this->blendStart.data(), 2);
        cost += this->blendInfo.ray * fabs(this->blendInfo.angleEnd - this->blendInfo.angleStart);
        cost += euclideanDistance(this->blendEnd.data(), ending_node.data(), 2);
        return cost;
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
