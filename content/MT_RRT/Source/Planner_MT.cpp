/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../Header/Planner_MT.h"
#include <ctime>
#include <omp.h>
#include <iostream>
using namespace std;

namespace MT_RTT
{

	I_Planner_MT::I_Planner_MT(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads) :
		I_Planner(det_coeff, max_iter, handler) {

		this->set_Threads(N_threads);

	};

	void I_Planner_MT::set_Threads(const size_t& Threads) {

		if (Threads == 0)   this->Thread_numbers = omp_get_max_threads();
		else				this->Thread_numbers = Threads;

		if (this->Thread_numbers < 2) throw 0;
		if (this->Thread_numbers > 100) cout << "warning: number of required threads is suspiciously high\n";

	}

	void I_Planner_MT::Init_Single_Extension_battery(std::vector<Single_Extension_job>* battery, const std::vector<I_Tree*>& T, const Node_State& target) {

		battery->reserve(T.size());
		for (size_t k = 0; k < T.size(); k++) 
			battery->emplace_back(T[k], target, &this->Deterministic_coefficient, &this->Cumulate_sol);

	};

	void I_Planner_MT::Init_Bidirectional_Extension_battery(std::vector<Bidirectional_Extension_job>* battery, const std::vector<I_Tree*>& A, const std::vector<I_Tree*>& B) {

		if (A.size() != B.size()) throw 0;

		battery->reserve(A.size());
		for (size_t k = 0; k < A.size(); k++)
			battery->emplace_back(A[k], B[k], &this->Deterministic_coefficient, &this->Cumulate_sol);

	};

	std::vector<unsigned int> I_Planner_MT::random_seeds(const size_t& N_seeds) {
#ifndef DETER_SEED
		srand((unsigned int)time(NULL) * 1000);
#endif

		std::vector<unsigned int> S;
		S.reserve(N_seeds);
		for (size_t k = 0; k < N_seeds; k++)
			S.emplace_back((unsigned int)rand());
		return S;

	};

}