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
    /** @return "[xMin, yMin, xMax, yMax]" of the passed rectangle
	 */
    arrayJSON log(const Rectangle& rectangle);
}

#endif