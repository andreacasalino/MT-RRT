#include "../MT_RRT/Header/Path_Planner.h"
#ifdef _DEBUG
#pragma comment (lib, "Lib/MT_RRTd.lib")
#else
#pragma comment (lib, "Lib/MT_RRT.lib")
#endif // DEBUG

#include <iostream>
using namespace std;



void create_3gdl_Problem(Path_simple_Node_Factory** factory, list<VectorF>* Qo, list<VectorF>* Qf) {

	Robot_Capsules Rob("../../Data/Robot/", "Scara_3_gdl.txt");
	Obstacles_list Scene("../../Data/Scenarios/Scene_01/", "Scene_01_Spheres_04.txt");

	int ngdl = Rob.Get_Ngdl();
	float Passo = sqrtf((float)Rob.Get_Ngdl()) * 3.0f * (PI_float / 180.0f);

	float deterministic_coeff = 0.1f; // 0.2f;
	int max_iter = 2500; //5000;
	int N_threads = 4;

	float* qo = (float*)malloc(Rob.Get_Ngdl() * sizeof(float));
	qo[0] = 30.0f * (PI_float / 180.0f);
	qo[1] = 75.0f * (PI_float / 180.0f);
	qo[2] = 50.0f * (PI_float / 180.0f);
	Qo->push_back(VectorF());
	Qo->back().copy(qo, ngdl);

	float* qf = (float*)malloc(Rob.Get_Ngdl() * sizeof(float));
	qf[0] = 15.0f * (PI_float / 180.0f);
	qf[1] = -90.0f * (PI_float / 180.0f);
	qf[2] = -50.0f * (PI_float / 180.0f);
	Qf->push_back(VectorF());
	Qf->back().copy(qf, ngdl);

	*factory = new Path_simple_Node_Factory(&Rob, &Scene, Passo, 4);

}


////////////////////////////////////////////////
// Basic example of usage, 
// considering the standard serial version
//
// Planning problem for a planar 3 gdl robot 
///////////////////////////////////////////////

int main() {
	// Define a node factory.
	// A node factory encapsulates all the informations required to make a steering procedure, as well check the admittance of a state.
	// Such a facory is actually then computed in the function create_3gdl_Problem
	Path_simple_Node_Factory* factory;
	list<VectorF> Qo, Qf; // the starting and the ending pose considered by the planning problem
	// creation of the planning problem
	create_3gdl_Problem(&factory, &Qo, &Qf);
	unsigned int max_iter = 2500;
	float deterministic_coeff = 0.1f;
	Planner_basic* solver;


	//////////////////
	// standard RRT //
	//////////////////

	//creation of the solver (the algorithm is not automatically launched when creating the solver)
	solver = new Planner_basic(Qo, Qf, factory, deterministic_coeff, max_iter, expansion_info::expansion_mode::single_tree, Planner_basic::Get_Handler(Planner_basic::Serial, 1));
	// execution of the RRT algorithm
	solver->Compute_solution();
	cout << "cmpt time " << solver->Get_computation_time() << endl;
	// log the tree computed. Every line L-th of the produced file contains two states of the tree, written row-wise.
	// The first state is of line L-th is the L-th node of the tree, while the second is the node in the tree preceding node L-th (i.e. its father).
	solver->Disp_Tree("../../MT_RRT/Tree") ;
	// Get the solution found in a list of states (i.e. the path found)
	list<VectorF> solving_path;
	solver->Get_solution(solving_path);
	// log the solution found. Every state of the solving path is written row-wise in the file
	solver->Disp_solution("../../MT_RRT/path_single");
	delete solver;



	///////////////////////
	// bidirectional RRT //
	///////////////////////

	//same steps of above

	solver = new Planner_basic(Qo, Qf, factory, deterministic_coeff, max_iter, expansion_info::expansion_mode::bi_directional, Planner_basic::Get_Handler(Planner_basic::Serial, 1));
	//solver->Disable_termination();
	solver->Compute_solution();
	cout << "cmpt time " << solver->Get_computation_time() << endl;
	solver->Disp_Tree("../../MT_RRT/Tree_bid"); // in this case, both the starting and the ending tree are logged into two distinct file: Tree_bid_end and Tree_bid_start 
	solver->Disp_solution("../../MT_RRT/path_bid");
	delete solver;



	//////////
	// RRT* //
	//////////

	solver = new Planner_basic(Qo, Qf, factory, deterministic_coeff, max_iter, expansion_info::expansion_mode::star, Planner_basic::Get_Handler(Planner_basic::Serial, 1));
	solver->Compute_solution();
	cout << "cmpt time " << solver->Get_computation_time() << endl;
	solver->Disp_Tree("../../MT_RRT/Tree_star");
	solver->Disp_solution("../../MT_RRT/path_star");
	delete solver;

	system("pause");
	return 0;
}