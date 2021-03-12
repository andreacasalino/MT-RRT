/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../Gatherer.h"
#include <omp.h>

namespace mt::copied {
    Gatherer::Gatherer(const std::vector<TreePtr>& battery) {
        this->battery = cast(battery);
    };

    void Gatherer::gather() const {
        this->battery[omp_get_thread_num()]->gather();
    };

    std::vector<copied::TreeConcreteLinked*> Gatherer::cast(const std::vector<TreePtr>& battery) {
        std::vector<copied::TreeConcreteLinked*> casted;
        casted.reserve(battery.size());
        for (auto it = battery.begin(); it != battery.cend(); ++it) {
            casted.push_back(dynamic_cast<copied::TreeConcreteLinked*>(it->get()));
        }
        return casted;
    };


    GathererBid::GathererBid(const std::vector<TreePtr>& batteryA, const std::vector<TreePtr>& batteryB)
        : Gatherer(batteryA) {
        this->battery2 = cast(batteryB);
    };

    void GathererBid::gather() const {
        this->Gatherer::gather();
        this->battery2[omp_get_thread_num()]->gather();
    };
}