/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <TreeIterable.h>
#include <Error.h>

namespace mt {
    TreeIterable::TreeIterable(NodePtr root) {
        if (nullptr == root) {
            throw Error("null root is impossible for TreeIterable");
        }
        this->nodes.emplace_back(std::move(root));
    }

    Node* TreeIterable::add(NodePtr node) { 
        if(nullptr != node) {
            this->nodes.emplace_back(std::move(node));
            return this->nodes.back().get(); 
        } 
        return nullptr;
    };
}