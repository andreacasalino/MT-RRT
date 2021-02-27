/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Checker.h>
#include "Common.h"

namespace mt::sample::geometry {
	void SegmentPointChecker::check(const Segment& segment, const Point& point) {
		this->computeCoeff(segment, point);
		if ((this->coeff < 0.f) || (this->coeff > 1.f)) {
			float d1 = dot(segment.A.data(), point.data(), segment.A.data(), point.data());
			float d2 = dot(segment.B.data(), point.data(), segment.B.data(), point.data());
			if (d1 < d2) this->coeff = 0.f;
			else		 this->coeff = 1.f;
		}
		this->computeInfo(segment, point);
	}
}