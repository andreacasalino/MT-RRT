/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../TreeStarMaster.h"

namespace mt::multiag {
    TreeStarMaster::TreeStarMaster(const std::vector<ProblemPtr>& problems, NodePtr root)
        : TreeMaster(problems, std::move(root)) {
    }

    void TreeStarMaster::gather() {
        // gather slaves result, while doing rewirds
        throw 0; // still to implement
    }
}