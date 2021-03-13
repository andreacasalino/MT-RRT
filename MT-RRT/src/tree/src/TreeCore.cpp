/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <TreeCore.h>

namespace mt {
    TreeCore::TreeCore(NodePtr root, Problem& problem) 
        : TreeBase(std::move(root), problem) {
    }
}