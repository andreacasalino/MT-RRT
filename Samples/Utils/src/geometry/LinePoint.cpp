/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Checker.h>
#include "Common.h"

namespace mt::sample::geometry {
	void LinePointChecker::computeCoeff(const Segment& line, const Point& point) {
		this->coeff = dot(point.data(), line.A.data(), line.B.data(), line.A.data()) / 
					  dot(line.B.data(), line.A.data(), line.B.data(), line.A.data());
	}

	void LinePointChecker::computeInfo(const Segment& line, const Point& point) {
		convexComb(this->closestInA, line.A, line.B, this->coeff);
		this->closestInB = point;
		this->distance = dot(this->closestInA.data(), this->closestInB.data());
		this->distance = sqrtf(this->distance);
	}

	void LinePointChecker::check(const Segment& line, const Point& point) {
		this->computeCoeff(line, point);
		this->computeInfo(line, point);
	}
}