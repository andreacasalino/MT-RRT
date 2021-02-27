/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Error.h>

namespace mt {
    Error::Error(const std::string& what)
        : std::runtime_error(what) {
    }
}
