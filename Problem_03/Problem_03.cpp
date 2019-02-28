#include "../MT_RRT/Header/Path_Planner.h"
#include "../MT_RRT/Header/Node_Kyno.h"
#ifdef _DEBUG
#pragma comment (lib, "../MT_RRT/Lib/MT_RRTd.lib")
#else
#pragma comment (lib, "../MT_RRT/Lib/MT_RRT.lib")
#endif // DEBUG

#include <iostream>
using namespace std;



void create_Traj_Problem(Kyno_Node_Factory** factory, list<VectorF>* Qo, list<VectorF>* Qf) {

	Robot_Capsules Rob("../../Data/Robot/", "Scara_3_gdl.txt");
	Obstacles_list Scene("../../Data/Scenarios/Scene_01/", "Scene_01_Spheres_04.txt");

	int ngdl = Rob.Get_Ngdl();
	float explor_time = 0.05f;
	// Set the kynematic limitations (values are taken only as an example)
	VectorF Vel_max(1.0f, ngdl), Acc_max(0.5f, ngdl), Null_Vec(0.0f, ngdl);
	float deterministic_coeff = 0.1f; // 0.2f;
	int max_iter = 5000;

	float* qo = (float*)malloc(Rob.Get_Ngdl() * sizeof(float));
	qo[0] = 30.0f * (PI_float / 180.0f);
	qo[1] = 75.0f * (PI_float / 180.0f);
	qo[2] = 50.0f * (PI_float / 180.0f);
	*Qo = { VectorF(), VectorF() };
	Qo->front().copy(qo, ngdl); Qo->back().copy(Null_Vec);
	float* qf = (float*)malloc(Rob.Get_Ngdl() * sizeof(float));
	qf[0] = 15.0f * (PI_float / 180.0f);
	qf[1] = -90.0f * (PI_float / 180.0f);
	qf[2] = -50.0f * (PI_float / 180.0f);
	*Qf = { VectorF(), VectorF() };
	Qf->front().copy(qf, ngdl); Qf->back().copy(Null_Vec);
	*factory = new Kyno_Node_Factory(&Rob, &Scene, explor_time, 5, Vel_max, Acc_max);

}


////////////////////////////////////////////////
// Trajectory planning problem for a planar 3 gdl robot;
// performance analysis
///////////////////////////////////////////////

int main() {
	// Define a node factory (see Basic_sample.cpp).
	Kyno_Node_Factory* factory;
	list<VectorF> Qo, Qf;
	// creation of the planning problem
	create_Traj_Problem(&factory, &Qo, &Qf);
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
	solver = new Planner_basic(Qo, Qf, factory, deterministic_coeff, max_iter, mode, Planner_basic::Get_Handler(Planner_basic::Handler_type::Serial, 1));
	// disable the termination criteria
	solver->Disable_termination();
	solver->Compute_solution();
	cout << "cmpt time " << solver->Get_computation_time() << endl;
	delete solver;

	cout << "-----------------------\n";
	cout << "Parallelization of the querying activities \n\n";
	for (int k = 1; k < 4; k++) {
		solver = new Planner_basic(Qo, Qf, factory, deterministic_coeff, max_iter, mode, Planner_basic::Get_Handler(Planner_basic::Handler_type::query_parall, k + 1));
		// disable the termination criteria
		solver->Disable_termination();
		solver->Compute_solution();
		cout << "Threads: " << k + 1 << ";  cmpt time " << solver->Get_computation_time() << endl;
		delete solver;
	}

	cout << "-----------------------\n";
	cout << "Parallel execution on a shared tree \n\n";
	for (int k = 1; k < 4; k++) {
		solver = new Planner_basic(Qo, Qf, factory, deterministic_coeff, max_iter, mode, Planner_basic::Get_Handler(Planner_basic::Handler_type::shared_tree, k + 1));
		// disable the termination criteria
		solver->Disable_termination();
		solver->Compute_solution();
		cout << "Threads: " << k + 1 << ";  cmpt time " << solver->Get_computation_time() << endl;
		delete solver;
	}

	cout << "-----------------------\n";
	cout << "Parallel execution on private copies of the searching tree \n\n";
	for (int k = 1; k < 4; k++) {
		solver = new Planner_basic(Qo, Qf, factory, deterministic_coeff, max_iter, mode, Planner_basic::Get_Handler(Planner_basic::Handler_type::copies_tree, k + 1));
		// disable the termination criteria
		solver->Disable_termination();
		solver->Compute_solution();
		cout << "Threads: " << k + 1 << ";  cmpt time " << solver->Get_computation_time() << endl;
		delete solver;
	}

	cout << "-----------------------\n";
	cout << "Multi agents approach \n\n";
	for (int k = 1; k < 4; k++) {
		solver = new Planner_basic(Qo, Qf, factory, deterministic_coeff, max_iter, mode, Planner_basic::Get_Handler(Planner_basic::Handler_type::multi_agents, k + 1));
		// disable the termination criteria
		solver->Disable_termination();
		solver->Compute_solution();
		cout << "Threads: " << k + 1 << ";  cmpt time " << solver->Get_computation_time() << endl;
		delete solver;
	}

	system("pause");
	return 0;
}