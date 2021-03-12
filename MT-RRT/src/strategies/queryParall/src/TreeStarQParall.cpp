/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeStarQParall.h"
#include "../header/Query.h"

namespace mt::solver::qpar {
    TreeStarQPar::TreeStarQPar(NodePtr root, const std::vector<ProblemPtr>& problems)
        : TreeStar<TreeQPar>(std::move(root), problems) {
    }

    class Result : public Query {
    public:
        template<typename ... IterArgs>
        Result(Problem& problem, IterArgs ... args) 
            : Query(problem, args...) {
        };

        void operator()(const NodeState& state, const float& ray) const {
            float dist_att;
            while (this->iterator.get() != this->iterator.getEnd()) {
                dist_att = this->problem.getTrajManager()->cost2Go(this->iterator.get()->get()->getState(), state, true);
                if (dist_att <= ray) {
                    this->set.emplace(this->iterator.get()->get());
                }
                ++this->iterator;
            }
        };

        mutable std::set<Node*> set;
    };

    std::set<Node*> TreeStarQPar::nearSet(const NodeState& state) const {
        float ray = this->nearSetRay();
        std::vector<Result> results = make_results<Result>(this->problems, this->rend(), this->rbegin());
        std::vector<Job> jobs;
        jobs.reserve(this->problems.size());
        for (std::size_t k = 0; k < this->problems.size(); ++k) {
            Result* temp = &results[k];
            jobs.emplace_back([temp, &state, &ray]() { (*temp)(state, ray); });
        }
        this->pool->addJob(jobs);
        this->pool->wait();

        // reduction
        auto it = results.begin();
        std::set<Node*> nearSet = it->set;
        ++it;
        for (it; it != results.end(); ++it) {
            for (auto itt = it->set.begin(); itt != it->set.end(); ++itt) {
                nearSet.emplace(*itt);
            }
        }
        return nearSet;
    }
}