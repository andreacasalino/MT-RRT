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
    /** @brief Used to manipulate the ranndom seeds used by sampling engine
     */
    class SeedFactory {
    public:
       /** @brief called when a new engine is created
        */
        static void addToRegister(std::default_random_engine* engine);
       /** @brief called when an engine is destroyed
        */
        static void removeFromRegister(std::default_random_engine* engine);

       /** @brief generate new seeds for all the so far created engines
        */
        static void resetSeeds();

       /** @param pass always the same value at the beginning of your application
        * in order to produce the same results
        */
        static void resetGeneratorSeed(const std::uint32_t& seed);
    };
}

#endif