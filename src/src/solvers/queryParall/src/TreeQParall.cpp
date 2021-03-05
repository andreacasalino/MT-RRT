/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../TreeQParall.h"

namespace mt::qpar {
    TreeQPar::TreeQPar(const std::vector<ProblemPtr>& problems, NodePtr root)
        : TreeConcrete(*problems.front(), std::move(root))
        , problems(make_battery(problems)) {
        this->pool = std::make_shared<Pool>();
    }

    TreeQPar::TreeQPar(const TreeQPar& o, NodePtr root)
        : TreeConcrete(*o.problems.front(), std::move(root))
        , problems(o.problems)
        , pool(o.pool) {
    }

    // used to iterate the nodes in a tree from a parallel for
    class Incrementer {
    public:
        Incrementer(const mt::Nodes& nodes, const Nodes::const_reverse_iterator& delimiter, const std::size_t& startPos, const std::size_t& delta)
            : end(nodes.crend())
            , cursor(delimiter)
            , delta(startPos) {
            ++(*this);
            this->delta = delta;
        };

        Incrementer& operator++() {
            std::size_t k = 0;
            while ((k<this->delta) && (this->cursor != this->end)) {
                ++this->cursor;
                ++k;
            }
            return *this;
        };

        inline const mt::Nodes::const_reverse_iterator& get() { return this->cursor; };

    private:
        Nodes::const_reverse_iterator end;
        Nodes::const_reverse_iterator cursor;
        std::size_t delta;
    };

    class QueryResult {
    public:
        QueryResult(const TreeQPar& user, const std::size_t& pos, const Nodes::const_reverse_iterator& delimiter)
            : user(user)
            , position(pos)
            , delimiter(delimiter) {
        };

    protected:
        const TreeQPar& user;
        const std::size_t position;
        const Nodes::const_reverse_iterator delimiter;
    };

    template<typename Q>
    std::vector<Q> make_QueryResults(const TreeQPar& user, const Nodes::const_reverse_iterator& delimiter, const std::size_t& size) {
        std::vector<Q> results;
        results.reserve(size);
        for (std::size_t k = 0; k < size; ++k) {
            results.emplace_back(user, k, delimiter);
        }
        return results;
    };

    Node* TreeQPar::nearestNeighbour(const NodeState& state, const Nodes::const_reverse_iterator& delimiter) const {
        class Result : public QueryResult {
        public:
            Result(const TreeQPar& user, const std::size_t& pos, const Nodes::const_reverse_iterator& delimiter) : QueryResult(user, pos, delimiter) {};

            void operator()(const NodeState& state) const {
                Incrementer inc(user.nodes, delimiter, this->position, user.problems.size());
                float temp;
                while (inc.get() != user.nodes.crend()) {
                    temp = user.problems[this->position]->cost2Go(inc.get()->get()->getState(), state, true);
                    if (temp < this->cost) {
                        this->cost = temp;
                        this->node = inc.get()->get();
                    }
                    ++inc;
                }
            };

            mutable float cost = mt::traj::Trajectory::COST_MAX;
            mutable Node* node = nullptr;
        };
        std::vector<Result> results = make_QueryResults<Result>(*this, delimiter, this->problems.size());

        std::vector<Job> jobs;
        jobs.reserve(this->problems.size());
        for (std::size_t k = 0; k < this->problems.size(); ++k) {
            Result& temp = results[k];
            jobs.emplace_back([temp, &state]() { temp(state); });
        }
        this->pool->addJob(jobs);
        this->pool->wait();

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

    std::set<Node*> TreeQPar::nearSet(const NodeState& state, const Nodes::const_reverse_iterator& delimiter) const {
        class Result : public QueryResult {
        public:
            Result(const TreeQPar& user, const std::size_t& pos, const Nodes::const_reverse_iterator& delimiter) : QueryResult(user, pos, delimiter) {};

            void operator()(const NodeState& state, const float& ray) const {
                Incrementer inc(user.nodes, delimiter, this->position, user.problems.size());
                float dist_att;
                while (inc.get() != user.nodes.crend()) {
                    dist_att = user.problems[this->position]->cost2Go(inc.get()->get()->getState(), state, true);
                    if (dist_att <= ray) {
                        this->set.emplace(inc.get()->get());
                    }
                    ++inc;
                }
            };

            mutable std::set<Node*> set;
        };
        std::vector<Result> results = make_QueryResults<Result>(*this, delimiter, this->problems.size());

        float Tree_size = static_cast<float>(std::distance(delimiter, this->nodes.rend()));
        float ray = this->problem.getGamma() * powf(logf(Tree_size) / Tree_size, 1.f / static_cast<float>(this->problem.getProblemSize()));

        std::vector<Job> jobs;
        jobs.reserve(this->problems.size());
        for (std::size_t k = 0; k < this->problems.size(); ++k) {
            Result& temp = results[k];
            jobs.emplace_back([temp, &state, &ray]() { temp(state, ray); });
        }
        this->pool->addJob(jobs);
        this->pool->wait();

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