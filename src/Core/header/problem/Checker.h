/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_CHECKER_H
#define MT_RRT_CHECKER_H

#include <Node.h>
#include <memory>

namespace mt::problem {
    class Checker {
    public:
        virtual	~Checker() = default;

        virtual std::unique_ptr<Checker> copy() const = 0;

        virtual bool isNotAdmitted(const NodeState& state) = 0;

    protected:
        Checker() = default;
    };

    typedef std::unique_ptr<Checker> CheckerPtr;
}

#endif