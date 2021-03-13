/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Checker.h>
#include "Common.h"

namespace mt::sample::geometry {
	void LineLineChecker::computeCoeff(const Segment& lineA, const Segment& lineB) {
		float m3 = dot(lineA.B.data(), lineA.A.data(), lineB.B.data(), lineB.A.data());
		float m1 = dot(lineA.B.data(), lineA.A.data(), lineA.B.data(), lineA.A.data());
		float m2 = dot(lineB.B.data(), lineB.A.data(), lineB.B.data(), lineB.A.data());

		float delta = m1 * m2 - m3 * m3;

		if (fabs(delta) <= 1e-5) {
			this->parallelism = true;
			return;
		}
		else this->parallelism = false;

		float c1 = dot(lineB.A.data(), lineA.A.data(), lineA.B.data(), lineA.A.data());
		float c2 = dot(lineA.A.data(), lineB.A.data(), lineB.B.data(), lineB.A.data());

		this->coeffA = (c2 * m3 + c1 * m2) / delta;
		this->coeffB = (this->coeffA * m3 + c2) / m2;
	}

	void LineLineChecker::computeInfo(const Segment& lineA, const Segment& lineB) {
		convexComb(this->closestInA, lineA.A, lineA.B, this->coeffA);
		convexComb(this->closestInB, lineB.A, lineB.B, this->coeffB);
		this->distance = dot(this->closestInA.data(), this->closestInB.data());
		this->distance = sqrtf(this->distance);
	}

	void LineLineChecker::check(const Segment& lineA, const Segment& lineB) {
		this->computeCoeff(lineA, lineB);
		if (this->parallelism) {
			LinePointChecker temp;
			temp.check(lineA, lineB.A);
			this->coeffA = temp.getCoeff();
			this->coeffB = 0.f;
		}
		this->computeInfo(lineA, lineB);
	}
}