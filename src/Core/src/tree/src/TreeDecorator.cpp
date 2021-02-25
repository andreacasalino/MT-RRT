/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <TreeDecorator.h>
#include <Error.h>

namespace mt::solver::tree {
    TreeDecorator::TreeDecorator(TreePtr wrapped) {
        if (nullptr == wrapped) {
            throw Error("TreeDecorator can't wrap nullptr");
        }
        this->wrapped = std::move(wrapped);
    }
}