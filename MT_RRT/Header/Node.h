#ifndef __NODE_H__
#define __NODE_H__

#include "My_Algebra.h"
#include <list>
#include <fstream>
using namespace std;

//Nodes encapsulate all the information related to a certain state in the tree, for any kind of planning problems.
//You have to created only a derived class of I_Node_Factory for specifying the planner the problem you want to solve,
// while class Node already contain in a general format all the informations required to describe a state.
// In particular:
// -pFather is the father of the considered node in the tree
// -the value of the state is split into mPose and  mState_additional (usually you can put all values in mPose and consider an empty mState_additional)
// -mCost_traj_from_father is the cost related to the trajectory which leads from pFather to this Node
// -mLinked_copies and mNames are data to be manipulated only by the planner.
class Node
{
	friend class I_Node_Factory;
public:
	struct Node_data {
		Node_data() : pFather(NULL), mNames(NULL), mCost_traj_from_father(0.f) {};
		Node*				pFather;
		VectorF				mPose;
		list<VectorF>		mState_additional;
		float				mCost_traj_from_father;
		list<Node*>			mLinked_copies;		//usefull only for omp
		int*				mNames;				//usefull only for mpi 
	};
	~Node();
// debug
	void			Disp_State(ostream& file);
	void			Disp_father_State(ostream& file) { this->mData.pFather->Disp_State(file); };
// methods
	void			Cost_to_root(float* result);
	void			copy_State(VectorF* S); //only to extract path
	Node*			Get_father() { return this->mData.pFather; };
	int				Get_State_size();   //return dimension of entire state: mPose + mState_additional
	list<int>		Get_State_subsizes();
	void			Cumul_traj_from_father_cost(float* result) { *result += this->mData.mCost_traj_from_father; };//only for star search
	void			Set_Linked_copies(list<Node*>& l) { this->mData.mLinked_copies = l; };
	list<Node*>*	Get_Linked_copies() { return &this->mData.mLinked_copies; };
	struct H_star_search {
		friend class star_search_decorator;
		friend class MultiT_shared_list_star;
		friend class MultiT_copied_list_decorator_star;
		friend class MultiP_democratic_decorator_star;
		friend class MultiT_master_decorator;
	private:
		static void	Set_father(Node* involved, Node* new_fath, float& new_cost_from_father);
		static void	Set_father(Node* involved, Node* new_fath) { involved->mData.pFather = new_fath; }; // for substuting father after copying in omp planner version
	};
	struct H_MultiP {
		friend class MultiP_democratic_decorator;
		friend class MultiP_democratic_decorator_star;
		friend class MultiP_master_decorator;
		friend class MultiP_slave_decorator;
		friend class MultiT_master_decorator;
		friend class MultiT_slave_decorator;
	private:
		static void	Get_State_copy(Node* involved, list<float*>* container); //copy of the state, for using sending mpi
		static int*	Get_nameID(Node* involved) { return involved->mData.mNames; };
		static void Set_nameID(Node* involved,int* name) { involved->mData.mNames = name; };
	};
private:
// constructor
	Node*			copy_this();
// data
	Node_data		mData;
};



class I_Node_Factory //when deriving, override only the virtual methods
{
public:
// constructor
	virtual ~I_Node_Factory() {};
// methods
	void						Set_name_size(const int& new_size) { this->mName_size = new_size; };
	void						Get_name_size(int* size) { *size = mName_size; };

	virtual I_Node_Factory*		copy() = 0; //in case of multithreading, every thread has a own private copy of the factory describing the problem. Therefore, this method is adopted for copying the initial factory passed to the constructor of the planner
	virtual float				Get_Gamma() = 0; //for RRT* expansion, returns the gamma parameter, i.e. the one regulating the size of the Near set
	virtual Node*				New_root(list<VectorF>* State) = 0; // to built the root node of a tree
	virtual Node*				Random_node() = 0; // for obtaining a new random state (=Node)
	virtual void				Distance(Node* start, Node* ending_node, float* result) = 0; // eval the cost of the optimal trajectory leading from start to ending_node, ignoring any additional contraints
	virtual Node*				Steer(Node* start, Node* trg, bool* trg_reached) = 0;		 //return NULL in case expansion is not possible. Set trg_reached only in the case the expansion lead succesfully to the target state trg
	virtual bool				Are_Traj_symmetric() = 0; //true when the optimal traj A->B is equal to the optimal traj B->A 
	virtual void				Traj_to_target(Node* start, Node* trg, float* result) = 0; //her the constraints are verified. In case the traj start->trg is not coll free must return FLOAT_MAX

	Node*						create_from_info(Node* father, list<float*>& state_copy, list<int>& state_size, int* name_copy); //for MultiP
	void						Extract_path(Node* N, list<VectorF>& States);
	void						Extract_path(Node* S, Node* E, list<VectorF>& States);
	Node*						copy_this(Node* involved) { return involved->copy_this(); };
protected:
	I_Node_Factory() : mName_size(0) {};
// methods
	Node::Node_data*			Get_data(Node* n) { return &n->mData; };
	void						Use_dangerous_set_VectorF(VectorF* to_set, float* pQ, const int& Size) { VectorF::dangerous_callback.set(to_set,pQ, Size); }; //to let derived factory use the dangerous setter of VectorF
	float*						Use_dangerous_get_VectorF(VectorF* to_catch) { return VectorF::dangerous_callback.Get_Q(to_catch); };
// data
	int							mName_size;
};



class Node_Factory_4simulation : public I_Node_Factory {
public:
	Node_Factory_4simulation(I_Node_Factory* to_wrap) :pwrapped_factory(to_wrap->copy()), mCoeff_Simul(1) {};
	~Node_Factory_4simulation() { delete this->pwrapped_factory; };
// methods
	I_Node_Factory*		copy();
	float				Get_Gamma() { return this->pwrapped_factory->Get_Gamma(); };
	Node*				New_root(list<VectorF>* State) { return this->pwrapped_factory->New_root(State); }; // for the root node
	Node*				Random_node() { return this->pwrapped_factory->Random_node(); }; //for random node
	void				Distance(Node* start, Node* ending_node, float* result);
	Node*				Steer(Node* start, Node* trg, bool* trg_reached);		 //return NULL in case expansion is not possible
	bool				Are_Traj_symmetric() { return this->pwrapped_factory->Are_Traj_symmetric(); };
	void				Traj_to_target(Node* start, Node* trg, float* result);

	void				Set_coeff_simul(const unsigned int& coeff) { if (coeff == 0) abort(); this->mCoeff_Simul = coeff; };
private:
	I_Node_Factory*		pwrapped_factory;
	unsigned int		mCoeff_Simul;		//only to simulate computation time higher for nodes            
};



#include "../Header/Geometry/Robot.h"
#include "../Header/Geometry/Obstacle.h"

class Path_simple_Node_Factory : public I_Node_Factory
{
public:
// constructor
	Path_simple_Node_Factory(Robot_shapes* rob, Obstacles_list* obst, const float& passo, const int& explor) :
		pRob(rob->copy()), pObst(obst->copy()), mPasso(passo), mExploration_degree(explor) { };
	~Path_simple_Node_Factory() { delete this->pRob; delete this->pObst; };
// methods
	I_Node_Factory*		copy();
	float				Get_Gamma() { return 100.f*this->mPasso; }; //for star expansion, it depends on exploration size
	Node*				New_root(list<VectorF>* State); // for the root node
	Node*				Random_node(); //for random node
	void				Distance(Node* start, Node* ending_node, float* result);
	Node*				Steer(Node* start, Node* trg, bool* trg_reached);		 //return NULL in case expansion is not possible
	bool				Are_Traj_symmetric() { return true; };
	void				Traj_to_target(Node* start, Node* trg, float* result);
protected:
// data
	Robot_shapes*		pRob;
	Obstacles_list*		pObst;
	float				mPasso;
	int					mExploration_degree;
};

#endif