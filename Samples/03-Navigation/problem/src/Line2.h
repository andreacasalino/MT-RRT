/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_NAVIGATION_LINE2_H
#define MT_RRT_SAMPLE_NAVIGATION_LINE2_H

#include <trajectory/Line.h>

namespace mt::traj {
    class TargetStore {
    protected:
        TargetStore(const NodeState& target);

        NodeState targetState;
    };

    class Line2
        : TargetStore
        , public Line {
    public:
        Line2(const NodeState& start, const NodeState& target, const float& steerDegree);
    };
}

#endif