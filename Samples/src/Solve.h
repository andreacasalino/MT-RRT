/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#ifndef __PERFORM_SIMULATION_H__
#define __PERFORM_SIMULATION_H__

#include "../../MT_RRT/Header/Planner_MT.h"
#include "../../MT_RRT/Header/json.h"
#include <iostream>
#include <chrono>


/* This class measures the time elapsing from consecutive calling to get_elapsed_from_previous
*/
class time_counter {
public:
	time_counter() : last_call(std::chrono::system_clock::now()) { };

	double get_elapsed_from_previous() {

		auto temp_now = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = temp_now - this->last_call;
		double elap = elapsed_seconds.count();
		this->last_call = temp_now;
		return elap;

	};
private:
// data
	std::chrono::time_point<std::chrono::system_clock> last_call;
};



/* This function solves the passed problem using the three possible strategies: RRT single tree, RRT bidirectionl and RRT star and creates a json log file (containing the trees computed
and the solutions found) at the location specified by  file_name.
*/
std::string Solve_using_strategies(std::unique_ptr< MT_RTT::I_Planner>& solver, const MT_RTT::Array& Start, const MT_RTT::Array& End) {

	std::string f = "";

	f += "{";
	f += "\"Start\":" + MT_RTT::json_parser::load_JSON(&Start[0], Start.size()) + "\n";
	f += ",\"Target\":" + MT_RTT::json_parser::load_JSON(&End[0], End.size()) + "\n";

	solver->RRT_basic(Start, End);
	f += ",\"RRT\":{";
	f += "\"Trees\":" + solver->Get_Trees_as_JSON() + "\n";
	f += ",\"Solution\":" + solver->Get_Solution_as_JSON() + "\n";
	f += "}";

	//check whether the bidirectional strategy is possible. 
	//It is not possible only for non symmetric problem. If a problem is symmetric, Node::I_Node_factory::Get_symm_flag should returns true.
	//Moreover, the multi agents planner does not implent it
	try
	{
		solver->RRT_bidirectional(Start, End);
		f += ",\"RRT_bid\":{";
		f += "\"Trees\":" + solver->Get_Trees_as_JSON() + "\n";
		f += ",\"Solution\":" + solver->Get_Solution_as_JSON() + "\n";
		f += "}";
	}
	catch (const int&) { std::cout << "  bidirectional skipped  "; }

	solver->RRT_star(Start, End);
	f += ",\"RRT_star\":{";
	f += "\"Trees\":" + solver->Get_Trees_as_JSON() + "\n";
	f += ",\"Solution\":"+ solver->Get_Solution_as_JSON() + "\n";
	f += "}";

	f += "}";
	return f;

}


/* This function solves the passed problem using the three possible strategies: RRT single tree, RRT bidirectionl and RRT star for all the possible plannning strategies. For each planner, a different json file
is created in the folder at the location specified by folder. 
*/
template<typename Simplifier = MT_RTT::Neutral_Simplifier>
std::vector<std::string> Solve_using_planners_and_strategies(const size_t& Iteration_max, const float& deterministic_coeff, MT_RTT::Node::I_Node_factory* planning_problem, const std::vector<float>& Start, const std::vector<float>& End, const float& reall_perc = 0.1f) {

// build the starting and ending node according to the passed state
	MT_RTT::Array Start_state(&Start[0], Start.size());
	MT_RTT::Array End_state(&End[0], End.size());
	
	time_counter Counter;
	std::unique_ptr<MT_RTT::I_Planner> solver;
	std::vector<std::string> results;
	results.reserve(5);

	{
		Counter.get_elapsed_from_previous();
//standard serial version of the RRT
		std::cout << "starting serial solver     ";
		// build the solver
		solver = MT_RTT::I_Planner::Get_canonical(deterministic_coeff, Iteration_max, planning_problem);
		solver->Set_post_processer<Simplifier>();
		// compute a solution for the problem
		results.emplace_back(Solve_using_strategies(solver, Start_state, End_state));
		std::cout << "done, elapsed  time "  << Counter.get_elapsed_from_previous() << std::endl;
	}

	{
		Counter.get_elapsed_from_previous();
//parallelization of the query (Section 3.0.1 of the documentation)
		std::cout << "starting parallelized query     ";
		// build the solver. Number of threads is omitted: the maximal number of threads available will be used
		solver = MT_RTT::I_Planner::Get_query___parall(deterministic_coeff, Iteration_max, planning_problem);
		solver->Set_post_processer<Simplifier>();
		// compute a solution for the problem
		results.emplace_back(Solve_using_strategies(solver, Start_state, End_state));
		std::cout << "done, elapsed  time " << Counter.get_elapsed_from_previous() << std::endl;
	}

	{
		Counter.get_elapsed_from_previous();
//parallel explorations on a shared tree (Section 3.0.2 of the documentation)
		std::cout << "starting parallel explorations on a shared tree     ";
		// build the solver. Number of threads is omitted: the maximal number of threads available will be used
		solver = MT_RTT::I_Planner::Get_shared__parall(deterministic_coeff, Iteration_max, planning_problem);
		solver->Set_post_processer<Simplifier>();
		// compute a solution for the problem
		results.emplace_back(Solve_using_strategies(solver, Start_state, End_state));
		std::cout << "done, elapsed  time " << Counter.get_elapsed_from_previous() << std::endl;
	}

	{
		Counter.get_elapsed_from_previous();
//parallel explorations in distributed copies (Section 3.0.3 of the documentation)
		std::cout << "starting parallel explorations on distributed copies     ";
		// build the solver. Number of threads is omitted: the maximal number of threads available will be used. The reallignement_percentage is omitted, 10% is assumed
		solver = MT_RTT::I_Planner::Get_copied__parall(deterministic_coeff, Iteration_max, planning_problem, 0, reall_perc);
		solver->Set_post_processer<Simplifier>();
		// compute a solution for the problem
		results.emplace_back(Solve_using_strategies(solver, Start_state, End_state));
		std::cout << "done, elapsed  time " << Counter.get_elapsed_from_previous() << std::endl;
	}

	{
		Counter.get_elapsed_from_previous();
//parallel explorations in distributed copies (Section 3.0.4 of the documentation)
		std::cout << "starting parallel multi agent     ";
		// build the solver. Number of threads is omitted: the maximal number of threads available will be used. The reallignement_percentage is omitted, 10% is assumed
		solver = MT_RTT::I_Planner::Get_multi_ag_parall(deterministic_coeff, Iteration_max, planning_problem, 0, reall_perc);
		solver->Set_post_processer<Simplifier>();
		// compute a solution for the problem
		results.emplace_back(Solve_using_strategies(solver, Start_state, End_state));
		std::cout << "done, elapsed  time " << Counter.get_elapsed_from_previous() << std::endl;
	}

	return results;

};

/* This function keep solve the same problem multiple times, returning the computational times
*/
std::vector<float> Solve_using_trials(MT_RTT::I_Planner* solver, const size_t& N_trial, const std::vector<float>& Qo, const std::vector<float>& Qf, const size_t& strategy) {

	std::vector<float> times;
	MT_RTT::Array No(&Qo[0], Qo.size()), Nf(&Qf[0], Qf.size());
	time_counter counter;
	for (size_t k = 0; k < N_trial; k++) {
		counter.get_elapsed_from_previous();
		if (strategy == 0)
			solver->RRT_basic(No, Nf);
		else if (strategy == 1) {
			try { solver->RRT_bidirectional(No, Nf); }
			catch (const int&) {}
		}
		else if (strategy == 2)
			solver->RRT_star(No, Nf);
		times.push_back((float)counter.get_elapsed_from_previous());

		std::cout << "\r trial: " << k;
	}
	return times;

}

/* This function profiles a specific multi threaded planner, evaluating the computational times when varying the number of adopted threads.
*/
std::vector<std::vector<float >> Solve_using_trials_threads(MT_RTT::I_Planner_MT* solver, const std::list<size_t>& threads, const size_t& N_trial, const std::vector<float>& Qo, const std::vector<float>& Qf, const size_t& strategy) {

	solver->Cumulate_solutions();
	time_counter counter;
	std::vector<std::vector<float>> temp;
	temp.reserve(threads.size());
	for (auto it = threads.begin(); it != threads.end(); it++) {
		std::cout << "thread: " << *it << std::endl;
		solver->set_Threads(*it);
		temp.push_back(Solve_using_trials(solver, N_trial, Qo, Qf, strategy));
	}
	return temp;

}


std::vector<float> import_pose(const std::vector<std::vector<float>>& vals) {

	std::vector<float> v;
	size_t K = 0;
	for (auto it = vals.begin(); it != vals.end(); it++) K += it->size();
	v.reserve(K);
	for (auto it = vals.begin(); it != vals.end(); it++) {
		for (auto it2 = it->begin(); it2 != it->end(); it2++)
			v.emplace_back(*it2 * 3.14159f / 180.f);
	}
	return v;

}


#endif