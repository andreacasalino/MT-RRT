#ifndef __NODE_KYNO_H__
#define __NODE_KYNO_H__


#include "Reflexxes/include/ReflexxesAPI.h"
#include "Reflexxes/include/RMLPositionFlags.h"
#include "Reflexxes/include/RMLPositionInputParameters.h"
#include "Reflexxes/include/RMLPositionOutputParameters.h"
#include "Node.h"

class Kyno_Node_Factory : public I_Node_Factory
{
public:
// constructor
	Kyno_Node_Factory(Robot_shapes* rob, Obstacles_list* obst, const float& sample_time, const int& explor_degree, VectorF& Vel_max, VectorF& Acc_max);
	I_Node_Factory*					copy();
	~Kyno_Node_Factory();
// methods
	float							Get_Gamma();
	Node*							New_root(list<VectorF>* State); // for the root node
	Node*							Random_node(); //for random node
	void							Distance(Node* start, Node* ending_node, float* result);
	Node*							Steer(Node* start, Node* trg, bool* trg_reached);		 //return NULL in case expansion is not possible
	bool							Are_Traj_symmetric() { return false; };
	void							Traj_to_target(Node* start, Node* trg, float* result); //collisions are verified. In case traj is not coll free return FLOAT_MAX

	void							Set_boundary(Node::Node_data* start_state, Node::Node_data* ending_state);
protected:
// data
	Robot_shapes*				 pRob;
	Obstacles_list*				 pObst;
	float						 mSample_time;
	int							 mExploration_degree;
	VectorF						 mVel_max;
	VectorF						 mAcc_max;

	int                          mResultValue;
	ReflexxesAPI*				 pRML;
	RMLPositionInputParameters*  pIP;
	RMLPositionOutputParameters* pOP;
	RMLPositionFlags			 mFlags;
};

#endif