#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

#include "Obstacle_Shapes.h"
#include "GJK.h"

class Obstacle
{
friend class Obstacles_list;
public:
	Obstacle(Convex_Shape* shape) : pShape(shape) {};
	~Obstacle() { delete pShape; };
// methods
	Obstacle* Copy();
private:
// data
	Convex_Shape*		pShape;
};

class Obstacles_list
{
public:
	static bool Initialize_Strategies();
// constructor
	Obstacles_list(std::string path_file, std::string name_file); //for reading all fixed obstacles of a scene
	Obstacles_list(list<Obstacle*> ob_list) : mObstacles(ob_list) {};
	Obstacles_list* copy();
	~Obstacles_list();
// methods
	bool Collision_absent(Obstacles_list* other_list);
private:
// constructor
	Obstacles_list() {}; //only for Obstacles_list::copy()
// methods, strategies
	struct IStrategy { virtual bool Check_Collision(Obstacle* Ob1, Obstacle* Ob2) = 0; }; //true if there is a collision
	struct GJK_strat : public IStrategy { bool Check_Collision(Obstacle* Ob1, Obstacle* Ob2); };
	struct ConvexShape_Plane : public IStrategy { bool Check_Collision(Obstacle* Ob1, Obstacle* Pln); };
	struct Plane_ConvexShape : public ConvexShape_Plane { bool Check_Collision(Obstacle* Pln, Obstacle* Ob1) { return this->ConvexShape_Plane::Check_Collision(Ob1, Pln); }; };
	struct Capsule_Capsule : public IStrategy { bool Check_Collision(Obstacle* Cap1, Obstacle* Cap2); };
	struct Capsule_Sphere : public IStrategy { bool Check_Collision(Obstacle* Cap, Obstacle* Sph); };
	struct Sphere_Capsule : public Capsule_Sphere { bool Check_Collision(Obstacle* Sph, Obstacle* Cap) { return this->Capsule_Sphere::Check_Collision(Cap, Sph); }; };
	struct Sphere_Sphere : public IStrategy { bool Check_Collision(Obstacle* Sphere1, Obstacle* Sphere2); };
	bool Check_between_shapes(Obstacle* Ob1, Obstacle* Ob2);
// data
	list<Obstacle*>				mObstacles;

	static bool					bStrategies_already_init;
	static IStrategy*			mStratetgy_Table[4][4];
};


#endif