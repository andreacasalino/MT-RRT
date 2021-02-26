/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../TreeQParall.h"

namespace mt::qpar {
    Tree::Tree(const std::vector<ProblemPtr>& problems, NodePtr root)
        : TreeConcrete(*problems.front(), std::move(root)) {
        this->problems.reserve(problems.size());
        for (std::size_t k = 0; k < problems.size(); ++k) {
            this->problems.push_back(problems[k].get());
        }
        this->pool = std::make_shared<Pool>();
    }

    Tree::Tree(const Tree& o, NodePtr root)
        : TreeConcrete(*o.problems.front(), std::move(root))
        , pool(o.pool) {
        this->problems = o.problems;
    }

    class Incrementer {
    public:
        Incrementer(const mt::Nodes& nodes, const std::size_t& startPos, const std::size_t& delta)
            : nodes(nodes) {
            this->cursor = nodes.begin();
            this->delta = startPos;
            ++(*this);
            this->delta = delta;
        };

        Incrementer& operator++() {
            this->cursorPos += this->delta;
            if (this->cursorPos < this->nodes.size()) {
                std::advance(this->cursor, this->delta);
            }
            else {
                this->cursor = this->nodes.end();
            }
            return *this;
        };

        inline const mt::Nodes::const_iterator& get() { return this->cursor; };

    private:
        const mt::Nodes& nodes;
        mt::Nodes::const_iterator cursor;
        std::size_t cursorPos = 0;
        std::size_t delta;
    };

    Node* Tree::nearestNeighbour(const NodeState& state) const {
        struct Result {
            float cost = mt::Trajectory::COST_MAX;
            Node* node = nullptr;
        };
        std::vector<Result> results;
        results.resize(this->problems.size());

        std::size_t id = 0;
        auto job = [this, &state, id, &results]() {
            Incrementer inc(this->nodes, id, this->problems.size());
            if (inc.get() == this->nodes.end()) return;
            results[id].cost = this->problems[id]->cost2Go(inc.get()->get()->getState(), state, true);
            results[id].node = inc.get()->get();
            float temp;
            ++inc;
            while (inc.get() != this->nodes.end()) {
                temp = this->problems[id]->cost2Go(inc.get()->get()->getState(), state, true);
                if (temp < results[id].cost) {
                    results[id].cost = temp;
                    results[id].node = inc.get()->get();
                }
                ++inc;
            }
        };
        std::vector<Pool::Job> jobs;
        jobs.reserve(this->problems.size());
        for (std::size_t k = 0; k < this->problems.size(); ++k) {
            jobs.emplace_back(job);
            ++id;
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
        return bestResult->node;
    }

    std::set<Node*> Tree::nearSet(Node& node) const {
        std::vector<std::set<Node*>> results;
        results.resize(this->problems.size());

        float Tree_size = static_cast<float>(this->nodes.size());
        float ray = this->problem.getGamma() * powf(logf(Tree_size) / Tree_size, 1.f / static_cast<float>(this->problem.getProblemSize()));

        std::size_t id = 0;
        auto job = [this, &node, id, &results, &ray]() {
            Incrementer inc(this->nodes, id, this->problems.size());
            float dist_att;
            while (inc.get() != this->nodes.end()) {
                dist_att = this->problems[id]->cost2Go(inc.get()->get()->getState(), node.getState(), true);
                if (dist_att <= ray) {
                    results[id].emplace(inc.get()->get());
                }
                ++inc;
            }
        };

        auto it = results.begin();
        std::set<Node*> nearSet = *it;
        ++it;
        for (it; it != results.end(); ++it) {
            for (auto itt = it->begin(); itt != it->end(); ++itt) {
                nearSet.emplace(*itt);
            }
        }
        return nearSet;
    }
}