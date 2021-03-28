/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TRAJECTORY_COMPOSITE_H
#define MT_RRT_TRAJECTORY_COMPOSITE_H

#include <trajectory/TrajectoryBase.h>
#include <list>

namespace mt::traj {
    /** @brief Base class for a Trajectory made of pieces of sub-ones.
	 */
    class TrajectoryComposite : public TrajectoryBase {
    public:
        inline NodeState getCursor() const override { return (*this->piecesCursor)->getCursor(); };

    protected:
        template<typename ... Pieces>
        TrajectoryComposite(Pieces&& ... piecess) {
            parsePieces(std::forward<Pieces>(piecess)...);

            for(auto it = this->pieces.begin(); it!=this->pieces.end(); ++it) {
                if(nullptr == *it) {
                    throw Error("found null piece for composite trajectory");
                }
            }
            this->piecesCursor = this->pieces.begin();
        };

        AdvanceInfo advanceInternal() override;

        std::list<TrajectoryPtr>           pieces;
        std::list<TrajectoryPtr>::iterator piecesCursor;
        Cost cumulatedCostPrevPieces; // previous pieces completely traversed

    private:
        template<typename ... Pieces>
        void parsePieces(TrajectoryPtr piece, Pieces&& ... piecess) {
            this->pieces.emplace_back(std::move(piece));
            parsePieces(std::forward<Pieces>(piecess)...);
        };
        void parsePieces(TrajectoryPtr piece) {
            this->pieces.emplace_back(std::move(piece));
        };
    };
}

#endif