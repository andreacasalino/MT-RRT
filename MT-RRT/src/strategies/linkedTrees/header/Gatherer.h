/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_GATHERER_H
#define MT_RRT_GATHERER_H

#include "TreeStarLinked.h"

namespace mt::solver::linked {
    class Gatherer {
    public:
        Gatherer(const std::vector<TreePtr>& battery);

        virtual void gather() const;

    protected:
        static std::vector<TreeLinked*> cast(const std::vector<TreePtr>& battery);

        std::vector<TreeLinked*> battery;
    };

    class GathererBid : public Gatherer {
    public:
        GathererBid(const std::vector<TreePtr>& batteryA, const std::vector<TreePtr>& batteryB);

        void gather() const override;

    private:
        std::vector<TreeLinked*> battery2;
    };
}

#endif