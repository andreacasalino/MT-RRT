/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_COPIABLE_H
#define MT_RRT_COPIABLE_H

#include <memory>

namespace mt {
    /** @brief Interface for a copiable object
	 */
    template<typename T>
    class Copiable {
    public:
        virtual ~Copiable() = default;

        virtual std::unique_ptr<T> copy() const = 0;

    protected:
        Copiable() = default;
    };
}

#endif
