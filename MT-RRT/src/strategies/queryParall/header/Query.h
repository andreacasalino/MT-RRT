/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_QUERY_H
#define MT_RRT_QUERY_H

#include "Incrementer.h"
#include <vector>

namespace mt::solver::qpar {
    class Query {
    public:
        template<typename ... IterArgs>
        Query(Problem& problem, IterArgs ... args) 
            : problem(problem), iterator(args...) {
        };

    protected:
        Problem& problem;
        mutable Incrementer iterator;
    };

    template<typename Q>
    std::vector<Q> make_results(const std::vector<Problem*>& problems, const Nodes::const_reverse_iterator& rend, const Nodes::const_reverse_iterator& rbegin) {
        std::vector<Q> results;
        results.reserve(problems.size());
        for(std::size_t k=0; k<problems.size(); ++k) {
            results.emplace_back(*problems[k], rend, rbegin, k, problems.size());
        }
        return results;
    }
}

#endif