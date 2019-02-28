#pragma once

#include "../MT_RRT/Header/Node.h"

#include <Core>
#include <Eigenvalues> 
using namespace Eigen;


void addition_Matrix_x_Vector(VectorF* result, VectorF& M, VectorF& V);
void subtraction_Matrix_x_Vector(VectorF* result, VectorF& M, VectorF& V);
void LQR(VectorF* Gain, const int& n, VectorF& A, float* T_close_loop);

class I_Dynamics_function { //dx=f(x)+u; linearized dx=A_linearized(x) + f_const + u
public:
	struct Constraint{ //Alfa*x + Beta*u <= Gamma 
	public:
		Constraint(MatrixXf& alfa, MatrixXf& beta, MatrixXf& gamma);
		Constraint(const Constraint& to_copy) {
			Alfa.copy(to_copy.Alfa);
			Beta.copy(to_copy.Beta);
			Gamma.copy(to_copy.Gamma);
		};

		// all the matrices are stored as vector row-wise (the conversion is done in the constructor)
		VectorF			Alfa;
		VectorF			Beta;
		VectorF			Gamma;
	};

	I_Dynamics_function(VectorF& xmin, VectorF& xmax); //no constraints
	I_Dynamics_function(VectorF& xmin, VectorF& xmax, Constraint& Cnstr);
	~I_Dynamics_function() { if (this->pCnstr != NULL) delete this->pCnstr; };
// methods
	virtual I_Dynamics_function* copy() = 0;
	virtual void eval_f(VectorF* f, VectorF& x) = 0; // f was already allocated with the rigth dimension
	virtual void eval_der_f(VectorF* A_linearized, VectorF& x) = 0;// A_linearized was already allocated with the rigth dimension
	virtual int  state_size() = 0;

	void Get_random_state(VectorF* x_rand);
	bool is_admitted(VectorF& x, VectorF& u);
	struct H_min_max {
		friend class SystemState_Factory;
	private:
		static void Get_extremals(I_Dynamics_function* to_extract, VectorF* xmin, VectorF* xmax);
	};
protected:
	I_Dynamics_function(I_Dynamics_function* to_copy);
// data
	VectorF				x_min;
	VectorF				x_delta;

	Constraint*			pCnstr;
};

class Non_Linear_system : public I_Dynamics_function {
public:
	Non_Linear_system(string& system_data, VectorF& xmin, VectorF& xmax, Constraint& Cnstr);
	Non_Linear_system(string& system_data, VectorF& xmin, VectorF& xmax);
	// methods
	I_Dynamics_function* copy();
	void eval_f(VectorF* f, VectorF& x);
	void eval_der_f(VectorF* A_linearized, VectorF& x);
	int  state_size() { return this->exp_coeff.Get_Size(); };
private:
	Non_Linear_system(VectorF& xmin, VectorF& xmax, VectorF& coeff, VectorF& t, VectorF& inv_t);

	void Init(string& system_data);
	// data
	VectorF exp_coeff;
	VectorF T;
	VectorF invT;
};

class SystemState_Factory : public I_Node_Factory
{
public:
// constructor
	SystemState_Factory(I_Dynamics_function* motion_equation, const float& dt, const unsigned int& explor_degree, const float& toll = 1e-4);

	I_Node_Factory*					copy();
	~SystemState_Factory() { delete this->pDyn_Syst; };
// methods
	float							Get_Gamma();
	Node*							New_root(list<VectorF>* State); // for the root node
	Node*							Random_node(); //for random node
	void							Distance(Node* start, Node* ending_node, float* result);
	Node*							Steer(Node* start, Node* trg, bool* trg_reached);		 //return NULL in case expansion is not possible
	bool							Are_Traj_symmetric() { return false; };
	void							Traj_to_target(Node* start, Node* trg, float* result); //collisions are verified. In case traj is not coll free return FLOAT_MAX
protected:
	SystemState_Factory() {};

	void compute_next_state(VectorF* state_attual, VectorF& dx, VectorF& u);
// data
	I_Dynamics_function*	pDyn_Syst;
	float					mDt;
	unsigned int			mExplor_degree;
	float					mToll;
};