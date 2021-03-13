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

    class Result : public Query {
    public:
        template<typename ... IterArgs>
        Result(Problem& problem, IterArgs ... args) 
            : Query(problem, args...) {
        };

        void operator()(const NodeState& state) const {
            float temp;
            while (this->iterator.get() != this->iterator.getEnd()) {
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
        std::vector<Result> results = make_results<Result>(this->problems, this->rend(), this->rbegin());
        std::vector<Job> jobs;
        jobs.reserve(this->problems.size());
        for (std::size_t k = 0; k < this->problems.size(); ++k) {
            Result* temp = &results[k];
            jobs.emplace_back([temp, &state]() { (*temp)(state); });
        }
        this->pool->addJob(jobs);
        this->pool->wait();

        // reduction
        auto it = results.begin();
        Result* bestResult = &(*it);
        ++it;
        for (it; it != results.end(); ++it) {
            if (it->cost < bestResult->cost) {
                bestResult = &(*it);
            }
        }
        if (nullptr == bestResult->node) return this->nodes.front().get();
        return bestResult->node;
    }
}