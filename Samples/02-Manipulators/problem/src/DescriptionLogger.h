/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_MANIPULATOR_DESCRIPTION_LOGGER_H
#define MT_RRT_SAMPLE_MANIPULATOR_DESCRIPTION_LOGGER_H

#include <ManipulatorProblem.h>
#include <SampleDescription.h>

namespace mt::sample {
    class DescriptionLogger
        : public sample::SampleDescription<sample::Description> {
    public:
        DescriptionLogger(const sample::Description& description);

        sample::structJSON logDescription() const override;
    };
}

#endif