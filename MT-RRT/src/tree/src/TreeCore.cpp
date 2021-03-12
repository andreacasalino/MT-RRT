/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <TreeCore.h>
#include <Error.h>

namespace mt {
    TreeCore::TreeCore(NodePtr root, Problem& problem)
        : problem(problem) {
        if (nullptr == root) {
            throw Error("null root is impossible for TreeCore");
        }
        this->nodes.emplace_back(std::move(root));
    }

    Node* TreeCore::add(NodePtr node) { 
        if(nullptr != node) {
            this->nodes.emplace_back(std::move(node));
            return this->nodes.back().get(); 
        } 
        return nullptr;
    };

    std::pair<NodePtr, bool> TreeCore::extend(const NodeState& target) {
		Node* nearest = this->nearestNeighbour(target);
		bool temp;
		NodePtr ext = this->getProblem().steer(*nearest, target, temp);
		return {std::move(ext) , temp};
	}

    Node* TreeCore::extendRandom() {
        auto temp = this->extend(this->getProblem().getSampler()->randomState());
        Node* pt = temp.first.get();
        this->add(std::move(temp.first));
        return pt;
    }

    Node* TreeCore::nearestNeighbour(const NodeState& state) const {
		auto it = this->rbegin();
		Node* nearest = it->get();
		const Problem& prb = this->getProblemConst();
		float nearestCost = prb.getTrajManager()->cost2Go((*it)->getState(), state, true), temp;
		++it;
        auto itEnd = this->rend();
		for (it; it != itEnd; ++it) {
			temp = prb.getTrajManager()->cost2Go((*it)->getState(), state, true);
			if (temp < nearestCost) {
				nearestCost = temp;
				nearest = (*it).get();
			}
		}
        return nearest;
    }
}