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
        for (std::size_t k = 0; k < problems.size(); ++k) {
            this->temporaryBuffers.emplace_back();
            this->computedRewirds.emplace_back();
        }
    }

    std::set<Node*> TreeStarMaster::nearSet(const NodeState& state) const {
        std::set<Node*> nearS = this->TreeRewirer::nearSet(state);
        std::size_t thId = omp_get_thread_num();
        const Problem* prb = this->slaves[thId]->getProblem();
		float ray = this->nearSetRay();
        float dist_att;
        auto itBf = this->temporaryBuffers.begin();
        std::advance(itBf, thId);
		for (auto itN = itBf->begin(); itN != itBf->end(); ++itN) {
			dist_att = prb->getTrajManager()->cost2Go((*itN)->getState(), state, true);
			if (dist_att <= ray) {
				nearS.emplace((*itN).get());
			}
		}
        return nearS;
    }

    void TreeStarMaster::gather() {
        // multi thread gather
        std::size_t thId = omp_get_thread_num();
        auto itBf = this->temporaryBuffers.begin();
        std::advance(itBf, thId);
        auto itRew = this->computedRewirds.begin();
        std::advance(itRew, thId);
        Nodes* nodes = this->slaves[thId]->getNodes();
        auto itN = nodes->begin();
        ++itN;
        for (itN; itN != nodes->end(); ++itN) {
            auto rew = this->computeRewires(**itN);
            for (auto it = rew.begin(); it != rew.end(); ++it) {
                itRew->push_back(*it);
            }
            itBf->emplace_back(std::move(*itN));
        }
#pragma omp barrier
        if (0 == thId) {
            for (auto it = this->slaves.begin(); it != this->slaves.end(); ++it) {
                Nodes* nodes = (*it)->getNodes();
                this->add(std::move(nodes->front()));
                nodes->clear();
            }

            for (auto it = this->temporaryBuffers.begin(); it != this->temporaryBuffers.end(); ++it) {
                for (auto itt = it->begin(); itt != it->end(); ++itt) {
                    this->add(std::move(*itt));
                }
                it->clear();
            }

            for (auto it = this->computedRewirds.begin(); it != this->computedRewirds.end(); ++it) {
                for (auto itt = it->begin(); itt != it->end(); ++itt) {
                    itt->involved.setFather(&itt->newFather, itt->newCostFromFather);
                }
                it->clear();
            }
        }
    }
}