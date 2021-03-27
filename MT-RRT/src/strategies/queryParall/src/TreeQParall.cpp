/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeQParall.h"
#include "../header/Query.h"

namespace mt::solver::qpar {
    TreeQPar::TreeQPar(NodePtr root, const std::vector<ProblemPtr>& problems)
        : TreeCore(std::move(root), *problems.front())
        , ProblemBattery(problems) {
        this->pool = std::make_shared<Pool>();
    }

    TreeQPar::TreeQPar(NodePtr root, const TreeQPar& o)
        : TreeCore(std::move(root), *o.problems.front())
        , ProblemBattery(o)
        , pool(o.pool) {
    }

    class QueryNeigh : public Query {
    public:
        QueryNeigh(Problem& problem, const TreeIterator& iterator) 
            : Query(problem, iterator) {
        };

        void operator()(const NodeState& state) const {
            float temp;
            while (this->iterator.get() != this->iterator.end()) {
                temp = this->problem.getTrajManager()->cost2Go(this->iterator.get()->get()->getState(), state, true);
                if (temp < this->cost) {
                    this->cost = temp;
                    this->node = this->iterator.get()->get();
                }
                ++this->iterator;
            }
        };

        mutable float cost = mt::traj::Cost::COST_MAX;
        mutable Node* node = nullptr;
    };

    Node* TreeQPar::nearestNeighbour(const NodeState& state) const {
        std::vector<QueryNeigh> results = make_results<QueryNeigh>(this->problems, *this);
        std::vector<Job> jobs;
        jobs.reserve(this->problems.size());
        for(auto it=results.begin(); it!=results.end(); ++it) {
            QueryNeigh* temp = &(*it);
            jobs.emplace_back([temp, &state]() { (*temp)(state); });
        }
        this->pool->addJob(jobs);
        this->pool->wait();

        // reduction
        auto it = results.begin();
        QueryNeigh* bestResult = &(*it);
        ++it;
        for (it; it != results.end(); ++it) {
            if (it->cost < bestResult->cost) {
                bestResult = &(*it);
            }
        }
        if (nullptr == bestResult->node) {
            return this->nodes.front().get();
        }
        return bestResult->node;
    }
}