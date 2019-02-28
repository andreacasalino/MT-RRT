#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "Obstacle.h"
#include "XML_importer.h"

class Robot {
public:
	// constructor
	Robot(std::string path_file, std::string name_file);
	Robot(XML_reader* reader) { this->Build_from_XML(reader); };
	Robot* copy();
	~Robot();
// methods
	Mrt*					Dir_Kyn(VectorF& Q);
	//Eigen::MatrixXd			J_pos(std::vector<Mrt>* M_chain, unsigned int k_chain);
// getters
	int						Get_Ngdl() { return this->mGDL; };
	void					Get_random_Pose(VectorF* containter);
protected:
	// auxiliary type
	struct DH;
	struct DH_rot;
	struct DH_lin;
// methods
	Robot() {};
	void Build_from_XML(XML_reader* reader);
	void copy_Robot(Robot* other);
// data
	Mrt						mBase;
	int						mGDL;
	DH**					pDH_params;
	VectorF					mQ_min;
	VectorF					mQ_max;
};

struct Robot::DH {
	DH() {};
	virtual void Get_Mrt(const float& q, Mrt& M_container) { abort(); };
	//virtual void Get_Jcol_pos(Mrt& M_container, Eigen::Vector3d& T_end, Eigen::Vector3d& j_pos) { abort(); };
	//virtual void Get_Jcol_w(Mrt& M_container, Eigen::Vector3d& T_end, Eigen::Vector3d& j_w) { abort(); };
	virtual DH* copy() { abort(); };
};

struct Robot::DH_rot : public Robot::DH {
	DH_rot() {};
	DH_rot(float off, float d_in, float a_in, float alf) : DH(), offset_teta(off), d(d_in), a(a_in), alfa(alf) {};
	virtual void Get_Mrt(const float& q, Mrt& M_container);
	//virtual void Get_Jcol_pos(Mrt& M_container, Eigen::Vector3d& T_end, Eigen::Vector3d& j_pos);
	//virtual void Get_Jcol_w(Mrt& M_container, Eigen::Vector3d& T_end, Eigen::Vector3d& j_w);
	virtual DH* copy();
// data
	float offset_teta;
	float d;
	float a;
	float alfa;
};

struct Robot::DH_lin : public Robot::DH {
	DH_lin() {};
	DH_lin(float tet, float off, float a_in, float alf) : DH(), teta(tet), offset_d(off), a(a_in), alfa(alf) {};
	virtual void Get_Mrt(const float& q, Mrt& M_container);
	//virtual void Get_Jcol_pos(Mrt& M_container, Eigen::Vector3d& T_end, Eigen::Vector3d& j_pos);
	//virtual void Get_Jcol_w(Mrt& M_container, Eigen::Vector3d& T_end, Eigen::Vector3d& j_w);
	virtual DH* copy();
// data
	float teta;
	float offset_d;
	float a;
	float alfa;
};



class Robot_shapes : public Robot
{
public:
	Robot_shapes(std::string path_file, std::string name_file);
	Robot_shapes* copy();
	~Robot_shapes() { delete this->pLinks_as_Obstacles; }; //destroy shapes
// methods
	void			Set_orientations(Mrt* chain);
	void			Set_orientations_byPose(VectorF& Q);
// getters
	Obstacles_list* Get_shapes() { return this->pLinks_as_Obstacles; };
protected:
	Robot_shapes() : Robot() {};
// methods
	void Build_Obstacles();
// data
	struct Link_shape { list<Convex_Shape*> shapes; }; //can be also empty
	Obstacles_list*		pLinks_as_Obstacles;
	list<Link_shape>	mLinks; 
};

class Robot_Capsules : public Robot_shapes
{
public:
// constructor
	Robot_Capsules(std::string path_file, std::string name_file);
};

#endif