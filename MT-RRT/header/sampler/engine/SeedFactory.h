/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SEED_FACTORY_H
#define MT_RRT_SEED_FACTORY_H

#include <random>

namespace mt::sampling {
    class SeedFactory {
    public:
        static void addToRegister(std::default_random_engine* engine);
        static void removeFromRegister(std::default_random_engine* engine);

        static void resetSeeds();
        static void resetGeneratorSeed(const std::uint32_t& seed);
    };
}

#endif