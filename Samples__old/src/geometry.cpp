/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "geometry.h"
#include <cmath>
#include <float.h>
#include <vector>
#include <iostream>
using namespace std;

Point3D::Point3D(const float& x, const float& y, const float& z) {
	this->coordinates[0] = x;
	this->coordinates[1] = y;
	this->coordinates[2] = z;
}

void Point3D::operator=(const Point3D& o) {
	for (size_t k = 0; k < 3; k++) this->coordinates[k] = o.coordinates[k];
}

void Point3D::Set_random() {
	for (size_t k = 0; k < 3; k++) this->coordinates[k] = -1.f + 2.f * (float)rand() / (float)RAND_MAX;
}

// result = (A1-A2)'*(B1-B2)
void Dot(float* result, const float* A1, const float* A2, const float* B1, const float* B2) {
	*result = (A1[0] - A2[0]) * (B1[0] - B2[0]);
	*result += (A1[1] - A2[1]) * (B1[1] - B2[1]);
	*result += (A1[2] - A2[2]) * (B1[2] - B2[2]);
}

void Dot(float* result, const float* A, const float* B) {
	Dot(result, A, B, A, B);
}

// result = A + (B-A)*s
void Convex_comb(Point3D* result, const Point3D& A, const Point3D& B, const float& s) {
	for (size_t k = 0; k < 3; k++) (*result)[k] = A[k] + s * (B[k] - A[k]);
}



void Line_VS_Point::__compute_s(const Segment& line, const Point3D& point) {

	float f1, f2;
	Dot(&f1, &point[0], &line.V1[0], &line.V2[0], &line.V1[0]);
	Dot(&f2, &line.V2[0], &line.V1[0], &line.V2[0], &line.V1[0]);
	this->s = f1 / f2;

}

void Line_VS_Point::__compute_info(const Segment& line, const Point3D& point) {

	Convex_comb(&this->closest_in_A, line.V1, line.V2, this->s);
	this->closest_in_B = point;
	Dot(&this->distance, &this->closest_in_A[0], &this->closest_in_B[0]);
	this->distance = sqrtf(this->distance);

}

void Line_VS_Point::Change_Pair(const Segment& line, const Point3D& point) {

	this->__compute_s(line, point);
	this->__compute_info(line, point);

}

void Segment_VS_Point::Change_Pair(const Segment& segment, const Point3D& point) {

	this->__compute_s(segment, point);
	if ((this->s < 0.f) || (this->s > 1.f)) {
		float d1, d2;
		Dot(&d1, &segment.V1[0], &point[0], &segment.V1[0], &point[0]);
		Dot(&d2, &segment.V2[0], &point[0], &segment.V2[0], &point[0]);
		if (d1 < d2) this->s = 0.f;
		else		 this->s = 1.f;
	}
	this->__compute_info(segment, point);

}



void Line_VS_Line::__compute_a_b(const Segment& lineA, const Segment& lineB) {

	float m3;
	Dot(&m3, &lineA.V2[0], &lineA.V1[0], &lineB.V2[0], &lineB.V1[0]);
	float m1, m2;
	Dot(&m1, &lineA.V2[0], &lineA.V1[0], &lineA.V2[0], &lineA.V1[0]);
	Dot(&m2, &lineB.V2[0], &lineB.V1[0], &lineB.V2[0], &lineB.V1[0]);

	float delta = m1 * m2 - m3 * m3;

	if (abs(delta) <= 1e-5) {
		this->are_parallel = true;
		return;
	}
	else this->are_parallel = false;

	float c1, c2;
	Dot(&c1, &lineB.V1[0], &lineA.V1[0], &lineA.V2[0], &lineA.V1[0]);
	Dot(&c2, &lineA.V1[0], &lineB.V1[0], &lineB.V2[0], &lineB.V1[0]);

	this->a = (c2 * m3 + c1 * m2) / delta;
	this->b = (this->a * m3 + c2) / m2;

}

void Line_VS_Line::__compute_info(const Segment& lineA, const Segment& lineB) {

	Convex_comb(&this->closest_in_A, lineA.V1, lineA.V2, this->a);
	Convex_comb(&this->closest_in_B, lineB.V1, lineB.V2, this->b);
	Dot(&this->distance, &this->closest_in_A[0], &this->closest_in_B[0]);
	this->distance = sqrtf(this->distance);

}

void Line_VS_Line::Change_Pair(const Segment& lineA, const Segment& lineB) {

	this->__compute_a_b(lineA, lineB);
	if (this->are_parallel) {
		Line_VS_Point temp(lineA, lineB.V1);
		this->a = temp.get_s();
		this->b = 0.f;
	}
	this->__compute_info(lineA, lineB);

}

void Segment_VS_Segment::Change_Pair(const Segment& segA, const Segment& segB) {

	this->__compute_a_b(segA, segB);
	if (this->are_parallel) this->__Check_vertices(segA, segB);
	else if ((this->a < 0.f) || (this->a > 1.f)|| (this->b < 0.f) || (this->b > 1.f))  this->__Check_vertices(segA, segB);
	else this->__compute_info(segA, segB);

}

void Segment_VS_Segment::__Check_vertices(const Segment& segA, const Segment& segB) {

	vector<Segment_VS_Point> checkers;
	checkers.reserve(4);
	checkers.emplace_back(segA, segB.V1);
	checkers.emplace_back(segA, segB.V2);
	checkers.emplace_back(segB, segA.V1);
	checkers.emplace_back(segB, segA.V2);

	size_t p_min = 0;
	for (size_t k = 1; k < 4; k++) {
		if (checkers[k].Get_distance() < checkers[p_min].Get_distance())
			p_min = k;
	}
	this->distance = checkers[p_min].Get_distance();
	if (p_min < 2) {
		this->closest_in_A = checkers[p_min].Get_closest_in_shapeA();
		this->closest_in_B = checkers[p_min].Get_closest_in_shapeB();
		this->a = checkers[p_min].get_s();
		if (p_min == 0) this->b = 0.f;
		else			this->b = 1.f;
	}
	else {
		this->closest_in_B = checkers[p_min].Get_closest_in_shapeA();
		this->closest_in_A = checkers[p_min].Get_closest_in_shapeB();
		this->b = checkers[p_min].get_s();
		if (p_min == 2) this->a = 0.f;
		else			this->a = 1.f;
	}

}

