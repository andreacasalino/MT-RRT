/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <TreeBase.h>
#include <Error.h>

namespace mt {
    TreeBase::TreeBase(NodePtr root, Problem& problem)
        : problem(&problem) {
        if (nullptr == root) {
            throw Error("null root is impossible for TreeBase");
        }
        this->nodes.emplace_back(std::move(root));
    }

    Node* TreeBase::add(NodePtr node) { 
        if(nullptr != node) {
            this->nodes.emplace_back(std::move(node));
            return this->nodes.back().get(); 
        } 
        return nullptr;
    };
}