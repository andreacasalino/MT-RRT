/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <TreeExtendable.h>

namespace mt {
    std::pair<NodePtr, bool> TreeExtendable::extend(const NodeState& target) {
		Node* nearest = this->nearestNeighbour(target);
		bool temp = false;
		NodePtr ext = this->getProblem()->steer(*nearest, target, temp);
		return {std::move(ext) , temp};
	}

    Node* TreeExtendable::nearestNeighbour(const NodeState& state) const {
		auto it = this->rbegin();
		Node* nearest = it->get();
		const Problem* prb = this->getProblem();
		float nearestCost = prb->getTrajManager()->cost2Go((*it)->getState(), state, true), temp;
		++it;
        auto itEnd = this->rend();
		for (it; it != itEnd; ++it) {
			temp = prb->getTrajManager()->cost2Go((*it)->getState(), state, true);
			if (temp < nearestCost) {
				nearestCost = temp;
				nearest = (*it).get();
			}
		}
        return nearest;
    }
}