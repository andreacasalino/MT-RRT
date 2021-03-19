/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_SPHERE_LOGGER_H
#define MT_RRT_SAMPLE_SPHERE_LOGGER_H

#include <Sphere.h>
#include <Logger.h>

namespace mt::sample::geometry {
    // [y,y,ray]
    arrayJSON log(const Sphere& sphere);
}

#endif