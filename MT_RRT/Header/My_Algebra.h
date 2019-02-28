#ifndef _MY_ALGEBRA__H_
#define _MY_ALGEBRA__H_

#include <list>
#include <stdlib.h>
#include <cmath>
#include <iostream>

struct RotMatrix;
struct Point3D 
{
// constructor
	Point3D() {}; //for building Obstacle
	Point3D(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {};
// methods
	void product_sx(const RotMatrix& rot, Point3D* result); //result = rot * this
	float squared_distance(const Point3D& other) { return (x - other.x)*(x - other.x) +
		(y - other.y)*(y - other.y) + (z - other.z)*(z - other.z); };
	float dot(const Point3D& other) { return (this->x*other.x + this->y*other.y + this->z*other.z); };
	void cross(const Point3D& other, Point3D* result);
	void operator *=(const float& coeff) { x *= coeff; y *= coeff; z *= coeff; };
	void operator +=(const Point3D& other) { x += other.x; y += other.y; z += other.z; };
	void operator -=(const Point3D& other) { x -= other.x; y -= other.y; z -= other.z; };
// data
	float x;
	float y;
	float z;
};

struct RotMatrix
{
	// methods
	void product_dx(const RotMatrix& other);
	void product(Point3D* P); //P=R*P
	void product_transpose(Point3D* P); //P=R'*P
										// data
	float R[3][3];
};

struct Mrt {
	// constructor
	Mrt() {}; //for robot constructor
	Mrt(const Mrt& to_copy) : mR(to_copy.mR), mT(to_copy.mT) {};
	Mrt(const Mrt* to_copy) : mR(to_copy->mR), mT(to_copy->mT) {};
	void copy(const Mrt* to_copy) { this->mR = to_copy->mR; this->mT = to_copy->mT; };
	// methods
	void product_dx(const Mrt& M2);
	// data
	RotMatrix mR;
	Point3D   mT;
};

struct VectorF
{
// constructor
	VectorF() : Q(NULL) {};
	VectorF(float* pQ,const int& Size) : Q_size(Size), Q(pQ) {};  //for generating random pose, reading Q_min/max and for MPI
	VectorF(const float& val, const int& Size);
	//all copy, allocate with malloc a new Q, and then copy every element of the object in input
	void copy(const VectorF& other); //to avoid destroy the pose when you do push_back
	void copy(const VectorF* other); 
	void copy(const Point3D& other);
	void copy(float* pQ,const int& Size);
	//VectorF(const VectorF& other) { std::cout << "Error, VectorF::VectorF(const VectorF& other) invoked \n "; abort(); }; //copy constructor, non si può disabilitare perchè serve a .push_back(VectorF())
	void operator = (const Point3D& other) { std::cout << "Error, VectorF::operator = (const VectorF& other) invoked \n "; abort(); };
	~VectorF() { free(this->Q); };
// methods
	void operator *=(const float& coeff);
	void operator +=(const VectorF& other);
	void operator -=(const VectorF& other);
	float& operator [](const int& pos) { return this->Q[pos]; };

	float L2_distance(const VectorF* other);
	float L1_distance(const VectorF* other);

	int	   Get_Size() { return this->Q_size; };
	void   Resize(const int& Size) { free(Q); Q_size = Size; Q = (float*)malloc(Q_size * sizeof(float)); };

	void set_zero();

	struct Dangerous_get_set { //it is a callback, to let only some specific classes access to the protected getter/setter
	friend class I_Node_Factory;
	friend class Node;
	friend class Robot;
	protected:
		float* Get_Q(VectorF* vector_considered) { return vector_considered->Q; };
		void set(VectorF* vector_considered, float* pQ, const int& Size) { free(vector_considered->Q); vector_considered->Q_size = Size; vector_considered->Q = pQ; };//without using malloc, mainly for MPI
	};
	static Dangerous_get_set dangerous_callback;
private:
	int Q_size;
	float* Q; //it is an array allocated with malloc
};


#endif