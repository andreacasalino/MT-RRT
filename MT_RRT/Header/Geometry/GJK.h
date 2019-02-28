
#ifndef __H_INCLUDE_GJK__
#define __H_INCLUDE_GJK__

#include "Obstacle_Shapes.h"

class GJK
{
public:
// constructor
	GJK(Convex_Shape* shape1, Convex_Shape* shape2);

	//~GJK();
// getters
	bool Are_in_collision() { return this->collision; };
private:
// methods
	void initial_loop();
	void compute_outside_normal(Point3D& N, Point3D& P1, Point3D& P2, Point3D& P3, Point3D& Pother);
	void Minkowski_difference(Point3D* result);
// data
	Convex_Shape*	C1;
	Convex_Shape*	C2;
	Point3D			Plex[4];
	Point3D			D;
	bool			collision; //if true there is a collision
};


#endif