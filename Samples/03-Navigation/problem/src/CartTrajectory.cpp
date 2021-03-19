/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "CartTrajectory.h"
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

    inline float atan2Delta (const float* pointA, const float* pointB) {
        return atan2(pointB[1] - pointA[1] , pointB[0] - pointA[0]);
    };
    TrajectoryPtr CartTrajectoryFactory::getTrajectory(const NodeState& start, const NodeState& target) const {
        float cosStart = cosf(start[2]);
        float sinStart = sinf(start[2]);
        float cosEnd   = cosf(target[2]);
        float sinEnd   = sinf(target[2]);

        sample::geometry::Point startVectorBegin (start[0], start[1]);
        sample::geometry::Point startVectorEnd (start[0] + cosStart, start[1] + sinStart);
        sample::geometry::Segment startVector(startVectorBegin, startVectorEnd);

        sample::geometry::Point endVectorBegin (target[0], target[1]);
        sample::geometry::Point endVectorEnd (target[0] + cosEnd, target[1] + sinEnd);
        sample::geometry::Segment endVector(endVectorBegin, endVectorEnd);

        sample::geometry::LineLineChecker checker;
        checker.check(startVector, endVector);

        if(checker.wereParallel()) {
            if(checker.getDistance() > CLOSENESS_TOLERANCE) {
                return nullptr;
            }
            float dot = cosStart * (target[0] - start[0]);
            dot += sinStart * (target[1] - start[1]);
            if(dot >= 0.0) {
                return std::make_unique<CartTrajectory>(std::make_unique<Line>(start , target, this->steerDegree) , &this->description);
            }
            return nullptr;
        }

        if(checker.getCoeffA() < 0.0) return nullptr;
        if(checker.getCoeffB() > 0.0) return nullptr;

        float angleMiddle = 0.5f * (target[3] - start[3]);
        float blendDistance = this->description.blendRadius / tanf(fabs(angleMiddle - target[2]));
        
        NodeState blendStart;
        NodeState blendEnd;
        if((squaredDistance(start.data(), checker.getClosesetInA().data(), 2) < blendDistance * blendDistance) || 
           (squaredDistance(target.data(), checker.getClosesetInA().data(), 2) < blendDistance * blendDistance)) {
            angleMiddle += 3.141f;
            blendStart = {checker.getClosesetInA().x() + cosStart * blendDistance , checker.getClosesetInA().y() + sinStart * blendDistance};
            blendEnd = {checker.getClosesetInA().x() - cosEnd * blendDistance     , checker.getClosesetInA().y() - sinEnd * blendDistance};
        }
        else {
            blendStart = {checker.getClosesetInA().x() - cosStart * blendDistance , checker.getClosesetInA().y() - sinStart * blendDistance};
            blendEnd = {checker.getClosesetInA().x() + cosEnd * blendDistance     , checker.getClosesetInA().y() + sinEnd * blendDistance};
        }
        CircleInfo blendInfo;
        blendInfo.angleStart = atan2Delta(checker.getClosesetInA().data(), blendStart.data());
        blendInfo.angleEnd = atan2Delta(checker.getClosesetInA().data(), blendEnd.data());
        blendInfo.ray = this->description.blendRadius;
        float centerDistance2Focal = this->description.blendRadius / sinf(angleMiddle);
        blendInfo.centerX = checker.getClosesetInA().x() + cosf(angleMiddle) * centerDistance2Focal;
        blendInfo.centerY = checker.getClosesetInA().y() + sinf(angleMiddle) * centerDistance2Focal;

        return std::make_unique<CartTrajectory>(std::make_unique<Line2>(start, blendStart, this->steerDegree), 
                                                std::make_unique<Circle>(blendInfo), 
                                                std::make_unique<Line>(blendEnd, target, this->steerDegree), &this->description);
    }

    CartTrajectory::CartTrajectory(std::unique_ptr<Line2> lineStart,std::unique_ptr<Circle> circle, std::unique_ptr<Line> lineEnd, const sample::Description* data) 
        : CartTrajectory(std::move(lineStart), data) {
        this->pieces.emplace_back(std::move(circle));
        this->pieces.emplace_back(std::move(lineEnd));
    }

    CartTrajectory::CartTrajectory(std::unique_ptr<Line> line, const sample::Description* data) {
        this->data = data;
        this->cumulatedCost.set(0.f);
        this->cumulatedCostContributions.push_back(0.f);
        this->pieces.emplace_back(std::move(line));
        this->piecesCursor = this->pieces.begin();
    }

    AdvanceInfo CartTrajectory::advance() {
        float prevCost = this->cumulatedCostContributions.back();
        auto temp = this->advanceNoCheck();
        for (auto it = this->data->obstacles.begin(); it != this->data->obstacles.end(); ++it) {
            if (this->data->cart.isColliding((*this->piecesCursor)->getCursor().data(), *it)) {
                temp = traj::AdvanceInfo::blocked;
                // std::swap(this->cursor, this->previousState);
                this->cumulatedCost.set(prevCost);
                break;
            }
        }
        return temp;
    };

    AdvanceInfo CartTrajectory::advanceNoCheck() {
        // this->previousState = this->getCursor();
        auto info = (*this->piecesCursor)->advance();
        if(AdvanceInfo::targetReached == info) {
            ++this->piecesCursor;
            if(this->piecesCursor == this->pieces.end()){
                --this->piecesCursor;
                return AdvanceInfo::targetReached;
            }
            else {
                this->cumulatedCostContributions.push_back(0.f);
            }
        }
        return info;
    }

    float CartTrajectory::sumCosts() const {
        auto it=this->cumulatedCostContributions.begin();
        float res = *it;
        ++it;
        for(it; it!=this->cumulatedCostContributions.end(); ++it) {
            res += *it;
        }
        return res;
    }
}
