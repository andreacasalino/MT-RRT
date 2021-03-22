/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <SphereLogger.h>

namespace mt::sample::geometry {
    arrayJSON log(const Sphere& sphere) {
        arrayJSON temp;
        temp.addElement(Number<float>(sphere.getCenter().x()));
        temp.addElement(Number<float>(sphere.getCenter().y()));
        temp.addElement(Number<float>(sphere.getRay()));
        return temp;
    };
}