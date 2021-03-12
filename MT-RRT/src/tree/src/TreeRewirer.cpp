/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <TreeRewirer.h>
#include <math.h>

namespace mt {

	float TreeRewirer::nearSetRay() const {
		float Tree_size = static_cast<float>(std::distance(this->getDelimiter(), this->getNodes().rbegin()));
		return this->getProblem().getGamma() * powf(logf(Tree_size) / Tree_size, 1.f / static_cast<float>(this->getProblem().getProblemSize()));
	}

    std::set<Node*> TreeRewirer::nearSet(const NodeState& state) const {
		const Problem& prb = this->getProblem();
		float ray = this->nearSetRay();
        float dist_att;
        std::set<Node*> nearS;
		for (auto itN = this->getDelimiter(); itN != this->getNodes().rend(); ++itN) {
			dist_att = this->getProblem().getTrajManager()->cost2Go((*itN)->getState(), state, true);
			if (dist_att <= ray) {
				nearS.emplace((*itN).get());
			}
		}
        return nearS;
    }

	Rewire::Rewire(Node& involved, Node& newFather, const float& newCostFromFather)
		: involved(involved)
		, newFather(newFather)
		, newCostFromFather(newCostFromFather) {
	}

    std::list<Rewire> TreeRewirer::computeRewires(Node& pivot) const {
		auto Near_set = this->nearSet(pivot.getState());
        {
            auto itNearest = Near_set.find(pivot.getFather());
            if (itNearest != Near_set.end()) {
                Near_set.erase(itNearest);
            }
        }
		if (Near_set.empty()) {
			return {};
		}

		const Problem& prb = this->getProblem();
		std::list<Rewire> rewirds;
		std::list<float> costs2RootNear_set;
		float costMin = mt::traj::Cost::COST_MAX, costAtt;
		std::list<Rewire>::iterator best_traj = rewirds.end();
		std::list<float>::iterator best_traj_cost2Root = costs2RootNear_set.end();
		for (auto itN = Near_set.begin(); itN != Near_set.end(); ++itN) {
			rewirds.emplace_front(**itN, pivot, prb.getTrajManager()->cost2Go((*itN)->getState(), pivot.getState(), false));
			if (rewirds.front().newCostFromFather == mt::traj::Cost::COST_MAX) {
				rewirds.pop_front();
			}
			else {
				costs2RootNear_set.push_front((*itN)->cost2Root());
				costAtt = rewirds.front().newCostFromFather + costs2RootNear_set.front();
				if (costAtt < costMin) {
					costMin = costAtt;
					best_traj = rewirds.begin();
					best_traj_cost2Root = costs2RootNear_set.begin();
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
			best_traj_cost2Root = costs2RootNear_set.begin();
		}
		else {
			best_traj->newFather.setFather(&best_traj->involved, best_traj->newCostFromFather);
		}
		rewirds.erase(best_traj);
		costs2RootNear_set.erase(best_traj_cost2Root);


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
					it_traj->newCostFromFather = prb.getTrajManager()->cost2Go(it_traj->newFather.getState(), it_traj->involved.getState(), false);
				}
				if (it_traj->newCostFromFather == mt::traj::Cost::COST_MAX) {
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