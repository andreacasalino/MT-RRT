/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <sampler/engine/SeedFactory.h>
#include <math.h>
#include <set>
#include <mutex>

namespace mt::sampling {
    std::mutex seedGeneratorMtx;

    std::default_random_engine seedGenerator;
    std::uniform_real_distribution<float> seedDistribution = std::uniform_real_distribution<float>(0.f, 1.f);

    std::set<std::default_random_engine*> engineRegister;

    void SeedFactory::resetGeneratorSeed(const std::uint32_t& seed) {
        std::lock_guard<std::mutex> lck(seedGeneratorMtx);
        seedGenerator.seed(seed);
    }

    void SeedFactory::resetSeeds() {
        std::lock_guard<std::mutex> lck(seedGeneratorMtx);
        for(auto it = engineRegister.begin(); it!=engineRegister.end(); ++it) {
            (*it)->seed( static_cast<std::uint32_t>( std::ceil( seedDistribution(seedGenerator) * 10000.0)) );
        }
    }

    void SeedFactory::addToRegister(std::default_random_engine* engine) {
        std::lock_guard<std::mutex> lck(seedGeneratorMtx);
        engineRegister.emplace(engine);
    }

    void SeedFactory::removeFromRegister(std::default_random_engine* engine) {
        std::lock_guard<std::mutex> lck(seedGeneratorMtx);
        engineRegister.erase(engineRegister.find(engine));
    }
}