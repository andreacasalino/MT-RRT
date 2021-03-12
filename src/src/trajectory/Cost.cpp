/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <trajectory/Cost.h>

namespace mt::traj {
    const float Cost::COST_MAX = std::numeric_limits<float>::max();

    Cost::Cost()
        : Positive<float>(COST_MAX) {
    }
}