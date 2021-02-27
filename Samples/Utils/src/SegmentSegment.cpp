/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Checker.h>
#include "Common.h"

namespace mt::sample::geometry {
	void SegmentSegmentChecker::check(const Segment& segA, const Segment& segB) {
		this->computeCoeff(segA, segB);
		if (this->parallelism) this->checkVertices(segA, segB);
		else if ((this->coeffA < 0.f) || (this->coeffA > 1.f) || (this->coeffB < 0.f) || (this->coeffB > 1.f))  this->checkVertices(segA, segB);
		else this->computeInfo(segA, segB);
	}

	void SegmentSegmentChecker::checkVertices(const Segment& segA, const Segment& segB) {
		SegmentPointChecker checkers[4];
		checkers[0].check(segA, segB.A);
		checkers[1].check(segA, segB.B);
		checkers[2].check(segB, segA.A);
		checkers[3].check(segB, segA.B);

		size_t p_min = 0;
		for (size_t k = 1; k < 4; ++k) {
			if (checkers[k].getDistance() < checkers[p_min].getDistance())
				p_min = k;
		}
		this->distance = checkers[p_min].getDistance();
		if (p_min < 2) {
			this->closestInA = checkers[p_min].getClosesetInA();
			this->closestInB = checkers[p_min].getClosesetInB();
			this->coeffA = checkers[p_min].getCoeff();
			if (p_min == 0) this->coeffB = 0.f;
			else			this->coeffB = 1.f;
		}
		else {
			this->closestInB = checkers[p_min].getClosesetInA();
			this->closestInA = checkers[p_min].getClosesetInB();
			this->coeffB = checkers[p_min].getCoeff();
			if (p_min == 2) this->coeffA = 0.f;
			else			this->coeffA = 1.f;
		}
	}
}