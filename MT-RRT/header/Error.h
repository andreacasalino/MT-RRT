/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_ERROR_H
#define MT_RRT_ERROR_H

#include <stdexcept>

namespace mt {
    /** @brief A runtime error that can be raised by any object inside mt::
	 */
    class Error : public std::runtime_error {
    public:
        explicit Error(const std::string& what);
    };
}

#endif
