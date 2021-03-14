/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeStarMaster.h"
#include <omp.h>

namespace mt::solver::multiag {
    TreeStarMaster::TreeStarMaster(NodePtr root, const std::vector<ProblemPtr>& problems)
        : TreeMaster(std::move(root), problems) {
        this->temporaryBuffer.resize(problems.size());
    }

    std::set<Node*> TreeStarMaster::nearSet(const NodeState& state) const {
        std::size_t thId = omp_get_thread_num();
        const Problem* prb = this->slaves[thId]->getProblem();
		float ray = this->nearSetRay();
        float dist_att;
        std::set<Node*> nearS;
		auto itEnd = this->rend();
		for (auto itN = this->rbegin(); itN != itEnd; ++itN) {
			dist_att = prb->getTrajManager()->cost2Go((*itN)->getState(), state, true);
			if (dist_att <= ray) {
				nearS.emplace((*itN).get());
			}
		}
		for (auto itN = this->temporaryBuffer[thId].begin(); itN != this->temporaryBuffer[thId].end(); ++itN) {
			dist_att = prb->getTrajManager()->cost2Go((*itN)->getState(), state, true);
			if (dist_att <= ray) {
				nearS.emplace((*itN).get());
			}
		}
        return nearS;
    }

    void TreeStarMaster::gather() {
        std::size_t thId = omp_get_thread_num();
        std::list<Rewire> rews;
        Nodes& nodes = this->slaves[thId]->getNodes();
        auto itN = nodes.begin();
        ++itN;
        for (itN; itN != nodes.end(); ++itN) {
            auto rew = this->computeRewires(**itN);
            for (auto it = rew.begin(); it != rew.end(); ++it) {
                rews.push_back(*it);
            }
            this->temporaryBuffer[thId].emplace_back(std::move(*itN));
        }
        nodes.clear();
        this->slaves[thId]->originalRoot = nullptr;
#pragma omp barrier
        if(0 == thId) {
            for(auto it = this->temporaryBuffer.begin(); it!=this->temporaryBuffer.end(); ++it) {
                for(auto itt = it->begin(); itt!=it->end(); ++itt) {
                    this->add(std::move(*itt));
                }
                it->clear();
            }
            for (auto it = rews.begin(); it != rews.end(); ++it) {
                it->involved.setFather(&it->newFather, it->newCostFromFather);
            }
        }
    }
}