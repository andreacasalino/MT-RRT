/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_UNIF_RAND_H
#define MT_RRT_UNIF_RAND_H

namespace mt {
    class UniformSampler {
    public:
        UniformSampler();

        UniformSampler(const unsigned int& s);

        float sample(const float& min, const float& delta);

    private:
        unsigned int seed;
    };
}

#endif
