#include "../Header/Extensions.h"
#include <iostream>
using namespace std;

namespace MT_RTT
{

	template<typename T>
	bool not_in_L(const T& to_find, const std::list<T>& L) {
		
		auto it_end = L.end();
		for (auto it = L.begin(); it != it_end; ++it) {
			if (*it == to_find)
				return false;
		}
		return true;

	}



	void	Single_Extension_job::Extend_within_iterations(const size_t& Iterations) {

		this->Check_Extension();

		float delta_cost;
		const Node* temp = nullptr;
		auto Problem = this->T->Get_Problem_Handler();
		for (size_t k = 0; k < Iterations; ++k) {
			if ((float)Problem->rand() / (float)RAND_MAX < *this->Deterministic_coefficient) {
				temp = T->Extend_deterministic(this->Target);
				if (T->Get_target_reached_flag()) {
					single_solution candidate(0.f, temp);
					this->A_solution_was_found = true;
					if (not_in_L(candidate, this->Solutions_found)) {
						temp->Cost_to_root(&candidate.cost);
						Problem->Cost_to_go(&delta_cost, temp, this->Target);
						candidate.cost += delta_cost;

						this->Solutions_found.emplace_back(candidate.cost, candidate.peer);
						if (!(*this->Cumulate_sol)) {
							this->Iterations_done = k;
							break;
						}
					}
				}
			}
			else  T->Extend_random();

			++this->Iterations_done;
#ifdef _DISPLAY_ITERATIONS
			cout << "iteration " << this->Iterations_done << endl;
#endif // _DISPLAY_ITERATIONS
		}

	};

	void Single_Extension_job::__Get_best_solution(std::list<Array>* solution, const std::list<single_solution>& solutions) {

		solution->clear();
		if (solutions.empty()) return;

		auto best_solution = get_best_solution(solutions);
		auto problem = this->T->Get_Problem_Handler();
		size_t S = problem->Get_State_size();

		solution->emplace_front(this->Target->Get_State(), S);
		const Node* cursor = best_solution->peer;
		while (cursor != nullptr) {
			solution->emplace_front(cursor->Get_State(), S);
			cursor = cursor->Get_Father();
		}

	}

	list<I_Tree*> Single_Extension_job::Remove_Trees() {
	
		I_Tree* temp = this->T;
		this->T = nullptr;
		return { temp }; 

	}



	void	Bidirectional_Extension_job::Extend_within_iterations(const size_t& Iterations) {

		this->Check_Extension();

		const Node* N_target = this->T_b->Get_root();
		bool caso = true;
		I_Tree* Master = this->T_a;
		I_Tree* Slave = this->T_b;
		I_Tree* temp;
		const Node* N1 = nullptr, * N2 = nullptr;
		bool dete_o_rand;
		bidir_solution candidate(0.f, nullptr, nullptr);
		for (size_t k = 0; k < Iterations; k += 2) {
			if ((float)Master->Get_Problem_Handler()->rand() / (float)RAND_MAX < *this->Deterministic_coefficient) {
				N1 = Master->Extend_deterministic(N_target);
				dete_o_rand = true;
			}
			else {
				N1 = Master->Extend_random();
				dete_o_rand = false;
			}

			if (N1 != nullptr) {
				if (Master->Get_target_reached_flag() && dete_o_rand) {
					this->A_solution_was_found = true;
					this->compute_sol(candidate, N1, N_target, caso);
					if (not_in_L(candidate, this->Solutions_found)) {
						this->compute_cost(candidate);
						this->Solutions_found.emplace_back(candidate.cost, candidate.peer_A, candidate.peer_B);
						if (!(*this->Cumulate_sol)) {
							this->Iterations_done = k;
							break;
						}
					}
				}
				else {
					N2 = Slave->Extend_deterministic(N1);
					if (Slave->Get_target_reached_flag()) {
						this->A_solution_was_found = true;
						this->compute_sol(candidate, N1, N2, caso);
						if (not_in_L(candidate, this->Solutions_found)) {
							this->compute_cost(candidate);
							this->Solutions_found.emplace_back(candidate.cost, candidate.peer_A, candidate.peer_B);
							if (!(*this->Cumulate_sol)) {
								this->Iterations_done = k;
								break;
							}
						}
					}
				}
			}

			temp = Master;
			Master = Slave;
			Slave = temp;
			caso = !caso;
			N_target = Slave->Get_root();

			++this->Iterations_done;
#ifdef _DISPLAY_ITERATIONS
			cout << "iteration " << this->Iterations_done << endl;
#endif // _DISPLAY_ITERATIONS
		}

	};

	void Bidirectional_Extension_job::__Get_best_solution(std::list<Array>* solution, const std::list<bidir_solution>& solutions) {

		solution->clear();
		if (solutions.empty()) return;
		auto best_solution = get_best_solution(solutions);
		auto problem = this->T_a->Get_Problem_Handler();
		size_t S = problem->Get_State_size();
		
		const Node* cursor = best_solution->peer_A;
		while (cursor != nullptr) {
			solution->emplace_front(cursor->Get_State(), S);
			cursor = cursor->Get_Father();
		}

		cursor = best_solution->peer_B;
		while (cursor != nullptr) {
			solution->emplace_back(cursor->Get_State(), S);
			cursor = cursor->Get_Father();
		}

	}

	void Bidirectional_Extension_job::compute_sol(bidir_solution& sol, const Node* N1, const Node* N2, const bool& caso) {

		if (caso) {
			sol.peer_A = N1;
			sol.peer_B = N2;
		}
		else {
			sol.peer_A = N2;
			sol.peer_B = N1;
		}

	}

	void Bidirectional_Extension_job::compute_cost(bidir_solution& sol) {

		sol.peer_A->Cost_to_root(&sol.cost);
		float delta;
		sol.peer_B->Cost_to_root(&delta);
		sol.cost += delta;
		auto problem = this->T_a->Get_Problem_Handler();
		problem->Cost_to_go(&delta, sol.peer_A, sol.peer_B);
		sol.cost += delta;

	}

	list<I_Tree*> Bidirectional_Extension_job::Remove_Trees() {

		list<I_Tree*> temp = {this->T_a, this->T_b};
		this->T_a = nullptr;
		this->T_b = nullptr;
		return temp;

	}

}