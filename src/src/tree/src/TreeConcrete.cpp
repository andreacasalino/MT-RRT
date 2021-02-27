/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <TreeConcrete.h>
#include <Error.h>

namespace mt {
    TreeConcrete::TreeConcrete(Problem& problem, NodePtr root)
        : problem(problem) {
        if (nullptr == root) {
            throw Error("null root is impossible for TreeConcrete");
        }
        this->nodes.emplace_back(std::move(root));
    }

    Node* TreeConcrete::extendRandom() {
		NodeState target = this->problem.randomState();
		Node* nearest = this->nearestNeighbour(target);
		bool temp;
		NodePtr ext = this->problem.steer(*nearest, target, temp);
		if (nullptr == ext) return nullptr;
		this->nodes.emplace_back(std::move(ext));
		return this->nodes.back().get();
    }

    std::pair<Node*, bool> TreeConcrete::extendDeterministic(const NodeState& target) {
        Node* nearest = this->nearestNeighbour(target);
        bool temp;
        NodePtr ext = this->problem.steer(*nearest, target, temp);
		if (temp) return {ext->getFather(), true};
        if (nullptr == ext) return {nullptr, false};
        this->nodes.emplace_back(std::move(ext));
        return { this->nodes.back().get(), false };
    }

    Node* TreeConcrete::nearestNeighbour(const NodeState& state) const {
		std::size_t treeSize = this->nodes.size();
        auto it = this->nodes.begin();
        Node* nearest = it->get();
        float nearestCost = this->problem.cost2Go(nearest->getState(), state, true), temp;
        ++it;
		for (std::size_t s = 1; s < treeSize; ++s) {
			temp = this->problem.cost2Go((*it)->getState(), state, true);
			if (temp < nearestCost) {
				nearestCost = temp;
				nearest = (*it).get();
			}
			++it;
		}
        return nearest;
    }

    std::set<Node*> TreeConcrete::nearSet(Node& node) const {
        float Tree_size = static_cast<float>(this->nodes.size());
        float ray = this->problem.getGamma() * powf(logf(Tree_size) / Tree_size, 1.f / static_cast<float>(this->problem.getProblemSize()));
        float dist_att;
        auto itN = this->nodes.begin();
        std::set<Node*> nearS;
        while (itN->get() != &node) {
            dist_att = this->problem.cost2Go((*itN)->getState(), node.getState(), true);
            if (dist_att <= ray) {
                nearS.emplace((*itN).get());
            }
            ++itN;
        }
        auto itNearest = nearS.find(node.getFather());
        if (itNearest != nearS.end()) {
            nearS.erase(itNearest);
        }
        return nearS;
    }

	TreeConcrete::Rewird::Rewird(Node& involved, Node& newFather, const float& newCostFromFather)
		: involved(involved)
		, newFather(newFather)
		, newCostFromFather(newCostFromFather) {
	}

    std::list<TreeConcrete::Rewird> TreeConcrete::computeRewirds(Node& pivot) const {
		std::list<TreeConcrete::Rewird> rewirds;
		auto Near_set = this->nearSet(pivot);
		if (Near_set.size() <= 1) {
			return rewirds;
		}

		std::list<float> costs2RootNear_set;
		float cost2RootMin = mt::traj::Trajectory::COST_MAX, costAtt;
		std::list<TreeConcrete::Rewird>::iterator best_traj = rewirds.end();
		for (auto itN = Near_set.begin(); itN != Near_set.end(); ++itN) {
			rewirds.emplace_front(**itN, pivot, this->problem.cost2Go((*itN)->getState(), pivot.getState(), false));
			if (rewirds.back().newCostFromFather == mt::traj::Trajectory::COST_MAX) {
				rewirds.pop_back();
			}
			else {
				costs2RootNear_set.push_front((*itN)->cost2Root());
				costAtt = rewirds.back().newCostFromFather + costs2RootNear_set.front();
				if (costAtt < cost2RootMin) {
					cost2RootMin = costAtt;
					best_traj = rewirds.begin();
				}
			}
		}
		// as a final iteration do the same with the current father
		rewirds.emplace_front(*pivot.getFather(), pivot, pivot.getCostFromFather());
		costs2RootNear_set.push_front(pivot.getFather()->cost2Root());
		costAtt = rewirds.back().newCostFromFather + costs2RootNear_set.front();
		if (costAtt < cost2RootMin) {
			cost2RootMin = costAtt;
			best_traj = rewirds.begin();
		}

		if (best_traj != rewirds.begin()) {
			best_traj->newFather.setFather(&best_traj->involved, best_traj->newCostFromFather);
		}
		rewirds.erase(best_traj);

		// check for rewird
		auto it_traj = rewirds.begin();
		auto it_cost2Root = costs2RootNear_set.begin();
		while (it_traj != rewirds.end()) {
			if (nullptr == it_traj->involved.getFather()) {
				it_traj = rewirds.erase(it_traj);
				it_cost2Root = costs2RootNear_set.erase(it_cost2Root);
			}
			else {
				if (!this->problem.isProblemSimmetric()) {
					it_traj->newCostFromFather = this->problem.cost2Go(it_traj->newFather.getState(), it_traj->involved.getState(), false);
				}
				if (it_traj->newCostFromFather == mt::traj::Trajectory::COST_MAX) {
					it_traj = rewirds.erase(it_traj);
					it_cost2Root = costs2RootNear_set.erase(it_cost2Root);
				}
				else {
					costAtt = cost2RootMin + it_traj->newCostFromFather;
					if (costAtt < *it_cost2Root) {
						++it_traj;
						++it_cost2Root;
					}
					else {
						it_traj = rewirds.erase(it_traj);
						it_cost2Root = costs2RootNear_set.erase(it_cost2Root);
					}
				}
			}
		}
		return rewirds;
    }
}