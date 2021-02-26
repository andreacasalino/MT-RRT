/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <cstdlib>

class Point3D {
public:
	Point3D() : Point3D(0.f, 0.f, 0.f) {};
	Point3D(const float& x, const float& y, const float& z);
	Point3D(const float& x, const float& y) : Point3D(x, y, 0.f) {};

	float& operator[](const size_t& pos) { return this->coordinates[pos]; };
	const float& operator[](const size_t& pos) const { return this->coordinates[pos]; };
	void operator =(const Point3D& o);

	void Set_random();
private:
// data
	float coordinates[3];
};
struct Segment {
	Point3D  V1;
	Point3D  V2;
};



class I_Checker {
public:
	const float& Get_distance() { return this->distance; };
	const Point3D& Get_closest_in_shapeA() { return this->closest_in_A; };
	const Point3D& Get_closest_in_shapeB() { return this->closest_in_B; };
protected:
// data
	float	distance;
	Point3D	closest_in_A;
	Point3D	closest_in_B;
};



class Line_VS_Point : public I_Checker {
public:
	Line_VS_Point(const Segment& line, const Point3D& point) { this->Change_Pair(line, point); };
	void Change_Pair(const Segment& line, const Point3D& point);

	//closest point to sphere is obtained as the following convex combination:
	//closest = seg.V1 + s * (seg.V2 - seg.V1)
	const float& get_s() { return s; };
protected:
	Line_VS_Point() {};
	void __compute_s(const Segment& line, const Point3D& point);
	void __compute_info(const Segment& line, const Point3D& point);
// data
	float		s;
};

class Segment_VS_Point : public Line_VS_Point {
public:
	Segment_VS_Point(const Segment& segment, const Point3D& point) { this->Change_Pair(segment, point); };
	void Change_Pair(const Segment& segment, const Point3D& point);
};



class Line_VS_Line : public I_Checker {
public:
	Line_VS_Line(const Segment& lineA, const Segment& lineB) { this->Change_Pair(lineA, lineB); };
	void Change_Pair(const Segment& lineA, const Segment& lineB);

	const bool& get_are_parallel() { return this->are_parallel; };

	//closest point in segA is obtained as the following convex combination:
	//closest = segA.V1 + a * (segA.V2 - segA.V1)
	const float& get_a() { return a; };
	//closest point in segB is obtained as the following convex combination:
	//closest = segB.V1 + b * (segB.V2 - segB.V1)
	const float& get_b() { return b; };
protected:
	Line_VS_Line() {};
	void __compute_a_b(const Segment& lineA, const Segment& lineB);
	void __compute_info(const Segment& lineA, const Segment& lineB);
// data
	bool		are_parallel;
	float		a;
	float		b;
};

class Segment_VS_Segment : public Line_VS_Line {
public:
	Segment_VS_Segment(const Segment& segA, const Segment& segB) { this->Change_Pair(segA, segB); };
	void Change_Pair(const Segment& segA, const Segment& segB);
protected:
	void __Check_vertices(const Segment& segA, const Segment& segB);
};

#endif