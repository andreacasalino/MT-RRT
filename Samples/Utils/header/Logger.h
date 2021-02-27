/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_LOGGER_H
#define MT_RRT_SAMPLE_LOGGER_H

#include <string>
#include <Solver.h>
#include <JSONstream.h>

namespace mt::sample {
    void addValues(arrayJSON& array, const float* data, const std::size_t& dataSize);

    class Logger {
    public:
        Logger(mt::Solver& solver);

        void print(const std::string& fileName);

        inline void addElement(const std::string& name, const streamJSON& json) { this->data.addElement(name, json); };

        inline void addEndl() { this->data.addEndl(); };

    private:
        mt::sample::structJSON data;
    };
}

#endif