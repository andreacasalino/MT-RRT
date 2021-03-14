/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_QUERY_H
#define MT_RRT_QUERY_H

#include "TreeIterator.h"
#include <vector>

namespace mt::solver::qpar {
    class Query {
    protected:
        Query(Problem& problem, const TreeIterator& iterator) 
            : problem(problem)
            , iterator(iterator) {
        };

        Problem& problem;
        mutable TreeIterator iterator;
    };

    template<typename Q>
    std::vector<Q> make_results(const std::vector<Problem*>& problems, const Tree& tree) {
        std::vector<Q> results;
        results.reserve(problems.size());
        for(std::size_t k=0; k<problems.size(); ++k) {
            results.emplace_back(*problems[k], TreeIterator(tree, k, problems.size()));
        }
        return results;
    }
}

#endif