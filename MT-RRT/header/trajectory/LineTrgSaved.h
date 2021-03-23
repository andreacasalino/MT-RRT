/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_LINE_TARGET_SAVED_H
#define MT_RRT_LINE_TARGET_SAVED_H

#include <trajectory/Line.h>

namespace mt::traj {
    class TargetStorer {
    protected:
        TargetStorer(const NodeState& target);

        const NodeState targetStored;
    };

    /** @brief Internally saves the target state, in order for the const refernce stored in Line
     * to remain meaningful.
	 */
    class LineTrgSaved
        : TargetStorer
        , public Line {
    public:
        LineTrgSaved(const NodeState& start, const NodeState& target, const float& steerDegree);
    };
}

#endif