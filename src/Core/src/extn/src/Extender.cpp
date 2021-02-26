/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Extender.h>

namespace mt {
    std::vector<NodeState> convert(const std::list<const NodeState*> nodes) {
        std::vector<NodeState> result;
        result.reserve(nodes.size());
        for (auto it = nodes.begin(); it != nodes.end(); ++it) {
            result.emplace_back(**it);
        }
        return result;
    }
}