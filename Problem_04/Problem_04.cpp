#include "../MT_RRT/Header/Path_Planner.h"
#ifdef _DEBUG
#pragma comment (lib, "../MT_RRT/Lib/MT_RRTd.lib")
#else
#pragma comment (lib, "../MT_RRT/Lib/MT_RRT.lib")
#endif // DEBUG

#include <iostream>
using namespace std;

#include "Node_Control.h"

void create_NonLinearProblem(SystemState_Factory** factory, const string& import_path, list<VectorF>* Xo, list<VectorF>* Xf) {

	I_Dynamics_function* eq_motion;
	float dt;
	int explor_degree = 15;

	dt = 0.005f;
	string temp_path(import_path);
	VectorF xmin(-2.f, 6), xmax(2.f, 6);

	MatrixXf Alfa_mat(12, 6); 
	Alfa_mat.setZero();
	MatrixXf Beta_mat(12, 6);
	Beta_mat.block(0, 0, 6, 6) = MatrixXf::Identity(6, 6);
	Beta_mat.block(6, 0, 6, 6) = -MatrixXf::Identity(6, 6);
	MatrixXf Gamma_mat(12, 1);
	Gamma_mat.setOnes();
	Gamma_mat *= 5.f;
	I_Dynamics_function::Constraint Cnstr(Alfa_mat, Beta_mat, Gamma_mat);
	
	eq_motion = new Non_Linear_system(temp_path, xmin, xmax, Cnstr);
	//eq_motion = new Non_Linear_system(temp_path, xmin, xmax); //for creating an unconstrained problem

	Xo->push_back(VectorF());
	Xf->push_back(VectorF());
	eq_motion->Get_random_state(&Xo->back());
	eq_motion->Get_random_state(&Xf->back());
	*factory = new SystemState_Factory(eq_motion, dt, explor_degree, 0.05f);

	delete eq_motion;

}


////////////////////////////////////////////////
// Kinodynamic planning problem for a non linear dynamical system;
// performance analysis
///////////////////////////////////////////////

int main() {

	list<VectorF> Xo, Xf;
	SystemState_Factory* factory;
	create_NonLinearProblem(&factory, "../../Problem_04/system_data", &Xo, &Xf);
	unsigned int max_iter = 2500;
	float deterministic_coeff = 0.1f;
	Planner_basic* solver;

	//set the RRT mode (single tree; bidirectional or RRT*)
	expansion_info::expansion_mode mode = expansion_info::expansion_mode::single_tree;

	//////////////////////////
	// Scalability analysis //
	//////////////////////////

	cout << "-----------------------\n";
	cout << "Serial version \n\n";
	solver = new Planner_basic(Xo, Xf, factory, deterministic_coeff, max_iter, mode, Planner_basic::Get_Handler(Planner_basic::Handler_type::Serial, 1));
	// disable the termination criteria
	solver->Disable_termination();
	solver->Compute_solution();
	cout << "cmpt time " << solver->Get_computation_time() << endl;
	delete solver;

	cout << "-----------------------\n";
	cout << "Parallelization of the querying activities \n\n";
	for (int k = 1; k < 4; k++) {
		solver = new Planner_basic(Xo, Xf, factory, deterministic_coeff, max_iter, mode, Planner_basic::Get_Handler(Planner_basic::Handler_type::query_parall, k + 1));
		// disable the termination criteria
		solver->Disable_termination();
		solver->Compute_solution();
		cout << "Threads: " << k + 1 << ";  cmpt time " << solver->Get_computation_time() << endl;
		delete solver;
	}

	cout << "-----------------------\n";
	cout << "Parallel execution on a shared tree \n\n";
	for (int k = 1; k < 4; k++) {
		solver = new Planner_basic(Xo, Xf, factory, deterministic_coeff, max_iter, mode, Planner_basic::Get_Handler(Planner_basic::Handler_type::shared_tree, k + 1));
		// disable the termination criteria
		solver->Disable_termination();
		solver->Compute_solution();
		cout << "Threads: " << k + 1 << ";  cmpt time " << solver->Get_computation_time() << endl;
		delete solver;
	}

	cout << "-----------------------\n";
	cout << "Parallel execution on private copies of the searching tree \n\n";
	for (int k = 1; k < 4; k++) {
		solver = new Planner_basic(Xo, Xf, factory, deterministic_coeff, max_iter, mode, Planner_basic::Get_Handler(Planner_basic::Handler_type::copies_tree, k + 1));
		// disable the termination criteria
		solver->Disable_termination();
		solver->Compute_solution();
		cout << "Threads: " << k + 1 << ";  cmpt time " << solver->Get_computation_time() << endl;
		delete solver;
	}

	cout << "-----------------------\n";
	cout << "Multi agents approach \n\n";
	for (int k = 1; k < 4; k++) {
		solver = new Planner_basic(Xo, Xf, factory, deterministic_coeff, max_iter, mode, Planner_basic::Get_Handler(Planner_basic::Handler_type::multi_agents, k + 1));
		// disable the termination criteria
		solver->Disable_termination();
		solver->Compute_solution();
		cout << "Threads: " << k + 1 << ";  cmpt time " << solver->Get_computation_time() << endl;
		delete solver;
	}

	system("pause");
	return 0;
}