/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TRAJECTORY_COMPOSITE_H
#define MT_RRT_TRAJECTORY_COMPOSITE_H

#include <trajectory/Trajectory.h>
#include <list>

namespace mt::traj {
    class TrajectoryComposite : public Trajectory {
    public:
        inline NodeState getCursor() const override { return (*this->piecesCursor)->getCursor(); };

        inline float getCumulatedCost() const override { return this->cumulatedCost.get() + (*this->piecesCursor)->getCumulatedCost(); };

        inline AdvanceInfo advance() override;

    protected:
        template<typename ... Pieces>
        TrajectoryComposite(Pieces ... otherPieces) {
            parsePieces(otherPieces...);

            for(auto it = this->pieces.begin(); it!=this->pieces.end(); ++it) {
                if(nullptr == *it) {
                    throw Error("found null piece for composite trajectory");
                }
            }
            this->piecesCursor = this->pieces.begin();
        };

        std::list<TrajectoryPtr>           pieces;
        std::list<TrajectoryPtr>::iterator piecesCursor;
        Cost cumulatedCost; // previous pieces completely traversed

    private:
        template<typename ... Pieces>
        void parsePieces(TrajectoryPtr piece, Pieces ... pieces) {
            this->pieces.emplace_back(std::move(piece));
            parsePieces(pieces...);
        };
        void parsePieces(TrajectoryPtr piece) {
            this->pieces.emplace_back(std::move(piece));
        };
    };
}

#endif