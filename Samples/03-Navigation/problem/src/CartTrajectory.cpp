/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "CartTrajectory.h"
#include <trajectory/Line.h>
#include <Checker.h>
#include <math.h>

namespace mt::traj {
    TrajectoryPtr CartTrajectory::make(const NodeState& start, const NodeState& target) {
        sample::geometry::Point startVectorBegin (start[0], start[1]);
        sample::geometry::Point startVectorEnd (start[0] + cosf(start[3]), start[1]+ sinf(start[3]));
        sample::geometry::Segment startVector(startVectorBegin, startVectorEnd);

        sample::geometry::Point endVectorBegin (target[0], target[1]);
        sample::geometry::Point endVectorEnd (target[0] + cosf(target[3]), target[1]+ sinf(target[3]));
        sample::geometry::Segment endVector(endVectorBegin, endVectorEnd);

        sample::geometry::LineLineChecker checker;
        checker.check(startVector, endVector);

        if() {

        }
        else if( (checker.getCoeffA() >= 0.0) && (checker.getCoeffB() <= 0.0) ) {

        }
        return nullptr;
    }

    CartTrajectory::CartTrajectory(const NodeState& start, TrajectoryPtr lineStart,TrajectoryPtr circle,TrajectoryPtr lineEnd) 
        : CartTrajectory(start, std::move(lineStart)) {
        this->pieces.emplace_back(std::move(circle));
        this->pieces.emplace_back(std::move(lineEnd));
        this->costs.push_back(0.f);
        // remove line if too short
        throw Error("not implemented");
    }

    CartTrajectory::CartTrajectory(const NodeState& start, TrajectoryPtr line) 
        : Trajectory(start) {
        this->cumulatedCost.set(0.f);
        this->pieces.emplace_back(std::move(line));
        this->piecesCursor = this->pieces.begin();
    }

    Trajectory::AdvanceInfo CartTrajectory::advance() {
        float prevCost = this->cumulatedCost.get();
        auto temp = this->advanceNoCheck();
        for (auto it = this->obstacles->begin(); it != this->obstacles->end(); ++it) {
            if (it->collideWithSegment(this->cursor.data(), this->previousState.data())) {
                temp = traj::Trajectory::AdvanceInfo::blocked;
                std::swap(this->cursor, this->previousState);
                this->cumulatedCost.set(prevCost);
                break;
            }
        }
        return temp;
    };

    Trajectory::AdvanceInfo CartTrajectory::advanceNoCheck() {
        std::swap(this->cursor, this->previousState);
        auto info = (*this->piecesCursor)->advance();
        this->cursor = (*this->piecesCursor)->getCursor();
        this->costs.back() = (*this->piecesCursor)->getCumulatedCost();
        this->cumulatedCost.set(this->sumCosts());
        if(Trajectory::AdvanceInfo::targetReached == info) {
            ++this->piecesCursor;
            if(this->piecesCursor == this->pieces.end()){
                --this->piecesCursor;
                return Trajectory::AdvanceInfo::targetReached;
            }
            else {
                this->costs.emplace_back(0.f);
                return Trajectory::AdvanceInfo::advanced;
            }
        }
        return info;
    }

    float CartTrajectory::sumCosts() const {
        float res = 0.f;
        for(auto it=this->costs.begin(); it!=this->costs.end(); ++it) {
            res += *it;
        }
        return res;
    }
}
