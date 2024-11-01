/**
 * Author:    Andrea Casalino
 * Created:   01.11.2024
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <chrono>

namespace mt_rrt {
template<typename ChronoT = std::chrono::nanoseconds>
struct ScopedTimeDuration {
    ScopedTimeDuration(ChronoT& recipient)
    : recipient_{recipient}
    , bornTime_{std::chrono::high_resolution_clock::now()}
    {}

    ~ScopedTimeDuration() {
        recipient_ = std::chrono::duration_cast<ChronoT>(std::chrono::high_resolution_clock::now() - bornTime_);
    }

private:
    ChronoT& recipient_;
    std::chrono::high_resolution_clock::time_point bornTime_;
};
}
