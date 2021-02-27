/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Point.h>
#include <math.h>

namespace mt::sample::geometry {
    // result = (A1-A2)'*(B1-B2)
    inline float dot(const float* A1, const float* A2, const float* B1, const float* B2) {
        float result = (A1[0] - A2[0]) * (B1[0] - B2[0]);
        result += (A1[1] - A2[1]) * (B1[1] - B2[1]);
        result += (A1[2] - A2[2]) * (B1[2] - B2[2]);
        return result;
    }

    inline float dot(const float* A, const float* B) {
        return dot(A, B, A, B);
    }

    // result = A + (B-A)*s
    inline void convexComb(Point& result, const Point& A, const Point& B, const float& s) {
        for (size_t k = 0; k < 3; k++) {
            result.data()[k] = A.data()[k] + s * (B.data()[k] - A.data()[k]);
        }
    }
}