#ifndef __PATH_PLANNER_H__
#define __PATH_PLANNER_H__

#include "Tree.h"
#include "Windows.h"
#include <time.h>
#include <omp.h>

#define PI_float 3.1415926f



class Planner_basic
{
public:
// constructor
	Planner_basic(list<VectorF>& start_state, list<VectorF>& end_state, I_Node_Factory* factory, float det_coeff, int max_iterations, expansion_info::expansion_mode mod, I_Tree_Handler* handler);
	expansion_info::expansion_mode Get_mod() { return this->pHndlr->Get_mode(); };
	~Planner_basic() { delete this->pHndlr; };
// methods
	enum Handler_type { Serial, query_parall, shared_tree, copies_tree, multi_agents };
	static I_Tree_Handler* Get_Handler(const Handler_type& hndl_type, const unsigned int& Threads_number); // in case hndl_type is Serial, Threads_number is ignored
	void Compute_solution();
	void Disable_termination() { this->pHndlr->Disable_termination(); };
// getters
	void Get_solution(list<VectorF>& percorso) { this->pHndlr->Get_solution(percorso); };
	int Get_iterations() { return this->mStats.iterations; };
	double Get_computation_time() { return this->mStats.Computation_time; };
// debug
	void Disp_solution(string file_traj); //virtual per fare check di essere in nodo 0 in mpi
	void Disp_Tree(string file_name); 
	//virtual void Disp_names(string file_name) { abort(); }; //only for mpi master_slaves version.
protected:
	struct statistics {
		int		iterations;
		double	Computation_time; };
// data
	I_Tree_Handler*			pHndlr;
	statistics				mStats;
	bool					bAlready_invoked;
};

#endif