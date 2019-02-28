#ifndef __OBSTACLE_SHAPE_H__
#define __OBSTACLE_SHAPE_H__

#include "../My_Algebra.h"
#include "XML_importer.h"
#include <vector>
#include <sstream>
using namespace std;

class Politope;
class Capsule;
class Sphere;
class Plane;
class Convex_Shape
{
public:
// methods
	virtual void Get_Support(Point3D d, Point3D* result, const bool& inverte_direction) = 0;
	virtual void Set_Orientation(const RotMatrix& rot, const Point3D& trsl) = 0;
	virtual Convex_Shape* copy() = 0;
// getters 
	virtual Politope*	Get_Politope_shape() { abort(); };
	virtual Capsule*	Get_Capsule_shape() { abort(); };
	virtual Sphere*		Get_Sphere_shape() { abort(); };
	virtual Plane*		Get_Plane_shape() { abort(); };
// data
	unsigned int		mID;
};

class Politope : public Convex_Shape {
	friend class Obstacles_list;
public:
	Politope(string name_file);
	Politope(vector<Point3D> cloud) : Clound_Point(cloud), Relative_Rotation(NULL), Relative_Traslation(NULL) { this->mID = 0; };
	~Politope(); //destroy ortientation data
// methods
	Convex_Shape* copy();
	void Get_Support(Point3D d, Point3D* result, const bool& inverte_direction); 
	void Set_Orientation(const RotMatrix& rot, const Point3D& trsl);
	Politope* Get_Politope_shape() { return this; };
private:
// data
	vector<Point3D>  Clound_Point;
	RotMatrix*	     Relative_Rotation;
	Point3D*		 Relative_Traslation;
};

class Capsule : public Convex_Shape {
	friend class Obstacles_list;
public:
	Capsule(Point3D& A, Point3D& B, const float& ray) : Relative_posA(A), Relative_posB(B), Ray(ray) { this->mID = 1; Absolute_posA = Relative_posA; Absolute_posB = Relative_posB; };
// methods
	Convex_Shape* copy();
	void Get_Support(Point3D d, Point3D* result, const bool& inverte_direction);
	void Set_Orientation(const RotMatrix& rot, const Point3D& trsl);
	Capsule* Get_Capsule_shape() { return this; };
private:
// data
	Point3D			Absolute_posA;
	Point3D			Absolute_posB;
	Point3D			Relative_posA;
	Point3D			Relative_posB;
	float			Ray;
};

class Sphere : public Convex_Shape {
	friend class Obstacles_list;
public:
	Sphere(Point3D& center, const float& ray) : Relative_Center(center), Ray(ray) { this->mID = 2; Absolute_Center = Relative_Center; };
// methods
	Convex_Shape* copy();
	void Get_Support(Point3D d, Point3D* result, const bool& inverte_direction);
	void Set_Orientation(const RotMatrix& rot, const Point3D& trsl);
	Sphere*		Get_Sphere_shape(){ return this; };
private:
// data
	Point3D			Absolute_Center;
	Point3D			Relative_Center;
	float			Ray;
};

class Plane : public Convex_Shape {
	friend class Obstacles_list;
public:
	Plane(Point3D& origin, Point3D& normal);
// methods
	Convex_Shape* copy();
	void Get_Support(Point3D d, Point3D* result, const bool& inverte_direction) { abort(); };
	void Set_Orientation(const RotMatrix& rot, const Point3D& trsl) { abort(); };
	Plane*	Get_Plane_shape() { return this; };
private:
// data
	Point3D&		Origin;
	Point3D&		Normal;
};

#endif