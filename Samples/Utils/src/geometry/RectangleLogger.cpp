/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <RectangleLogger.h>

namespace mt::sample::geometry {
    arrayJSON log(const Rectangle& rectangle) {
        arrayJSON temp;
        temp.addElement(Number<float>(rectangle.getXMin()));
        temp.addElement(Number<float>(rectangle.getYMin()));
        temp.addElement(Number<float>(rectangle.getXMax()));
        temp.addElement(Number<float>(rectangle.getYMax()));
        return temp;
    };
}