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

	std::pair<NodePtr, bool> TreeConcrete::extend(const NodeState& target) {
		Node* nearest = this->nearestNeighbour(target, this->getDelimiter());
		bool temp;
		NodePtr ext = this->getProblem().steer(*nearest, target, temp);
		return {std::move(ext) , temp};
	}

    Node* TreeConcrete::nearestNeighbour(const NodeState& state, const Nodes::const_reverse_iterator& delimiter) const {
		auto it = delimiter;
		Node* nearest = it->get();
		const Problem& prb = this->getProblem();
		float nearestCost = prb.cost2Go((*it)->getState(), state, true), temp;
		++it;
		for (it; it != this->nodes.rend(); ++it) {
			temp = prb.cost2Go((*it)->getState(), state, true);
			if (temp < nearestCost) {
				nearestCost = temp;
				nearest = (*it).get();
			}
		}
        return nearest;
    }

    std::set<Node*> TreeConcrete::nearSet(const NodeState& state, const Nodes::const_reverse_iterator& delimiter) const {
		float Tree_size = static_cast<float>(std::distance(delimiter, this->getDelimiter()));
		const Problem& prb = this->getProblem();
        float ray = prb.getGamma() * powf(logf(Tree_size) / Tree_size, 1.f / static_cast<float>(prb.getProblemSize() ));
        float dist_att;
        std::set<Node*> nearS;
		for (auto itN = delimiter; itN != this->nodes.rend(); ++itN) {
			dist_att = prb.cost2Go((*itN)->getState(), state, true);
			if (dist_att <= ray) {
				nearS.emplace((*itN).get());
			}
		}
        auto itNearest = nearS.find((*delimiter)->getFather());
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

    std::list<TreeConcrete::Rewird> TreeConcrete::computeRewirds(Node& pivot, const Nodes::const_reverse_iterator& delimiter) const {
		auto Near_set = this->nearSet(pivot.getState(), delimiter);
		if (Near_set.empty()) {
			return {};
		}

		const Problem& prb = this->getProblem();
		std::list<TreeConcrete::Rewird> rewirds;
		std::list<float> costs2RootNear_set;
		float costMin = mt::traj::Trajectory::COST_MAX, costAtt;
		std::list<TreeConcrete::Rewird>::iterator best_traj = rewirds.end();
		for (auto itN = Near_set.begin(); itN != Near_set.end(); ++itN) {
			rewirds.emplace_front(**itN, pivot, prb.cost2Go((*itN)->getState(), pivot.getState(), false));
			if (rewirds.front().newCostFromFather == mt::traj::Trajectory::COST_MAX) {
				rewirds.pop_front();
			}
			else {
				costs2RootNear_set.push_front((*itN)->cost2Root());
				costAtt = rewirds.front().newCostFromFather + costs2RootNear_set.front();
				if (costAtt < costMin) {
					costMin = costAtt;
					best_traj = rewirds.begin();
				}
			}
		}
		// as a final iteration do the same with the current father
		rewirds.emplace_front(*pivot.getFather(), pivot, pivot.getCostFromFather());
		costs2RootNear_set.push_front(pivot.getFather()->cost2Root());
		costAtt = rewirds.front().newCostFromFather + costs2RootNear_set.front();
		if (costAtt < costMin) {
			costMin = costAtt;
			best_traj = rewirds.begin();
		}
		else {
			best_traj->newFather.setFather(&best_traj->involved, best_traj->newCostFromFather);
		}
		rewirds.erase(best_traj);

		// check for rewird
		auto it_traj = rewirds.begin();
		auto it_cost2Root = costs2RootNear_set.begin();
		while (it_traj != rewirds.end()) {
			if (nullptr == it_traj->involved.getFather()) {
				// root can't be rewired
				it_traj = rewirds.erase(it_traj);
				it_cost2Root = costs2RootNear_set.erase(it_cost2Root);
			}
			else {
				if (!prb.isProblemSimmetric()) {
					it_traj->newCostFromFather = prb.cost2Go(it_traj->newFather.getState(), it_traj->involved.getState(), false);
				}
				if (it_traj->newCostFromFather == mt::traj::Trajectory::COST_MAX) {
					it_traj = rewirds.erase(it_traj);
					it_cost2Root = costs2RootNear_set.erase(it_cost2Root);
				}
				else {
					costAtt = costMin + it_traj->newCostFromFather;
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