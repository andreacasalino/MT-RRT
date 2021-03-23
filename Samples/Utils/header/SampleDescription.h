/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_DESCRIPTION_H
#define MT_RRT_SAMPLE_DESCRIPTION_H

#include <JSONstream.h>

namespace mt::sample {
    /** @brief Interface for an object storing the information that completely defines a planning problem
     */
    template<typename Description>
    class SampleDescription {
    public:
        SampleDescription(const Description& description) 
            : description(description) {
        };

        inline const Description& getDescription() const { return this->description; }

        virtual structJSON logDescription() const = 0;

    protected:
        const Description description;
    };
}

#endif