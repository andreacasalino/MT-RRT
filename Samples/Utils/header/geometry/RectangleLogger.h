/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_RECTANGLE_LOGGER_H
#define MT_RRT_SAMPLE_RECTANGLE_LOGGER_H

#include <Rectangle.h>
#include <Logger.h>

namespace mt::sample::geometry {
    // [xMin, yMin, xMax, yMax]
    arrayJSON log(const Rectangle& rectangle);
}

#endif