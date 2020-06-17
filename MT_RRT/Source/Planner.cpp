/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../Header/Planner.h"
#include "../Header/json.h"
using namespace std;

namespace MT_RTT
{

	void copy_solution(list<Node_State>& receiving, const list<Node_State>& sending) {

		receiving.clear();
		auto it_end = sending.end();
		for (auto it = sending.begin(); it != it_end; it++)
			receiving.emplace_back(*it);

	}



	I_Planner::I_Planner(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler) :
		Handler(handler), Deterministic_coefficient(det_coeff), Iterations_Max(max_iter), Cumulate_sol(false), Last_solution(nullptr) {

		if (this->Deterministic_coefficient > 1.f)
			this->Deterministic_coefficient = 1.f;
		if (this->Deterministic_coefficient < 0.f)
			this->Deterministic_coefficient = 0.01f;

	};

	I_Planner::~I_Planner() {

		if (this->Last_solution != nullptr) {
			this->__clean_trees();
			delete this->Last_solution;
		}

	}

	void I_Planner::__clean_trees() {
		
		auto it_end = this->Last_solution->Trees.end();
		for (auto it = this->Last_solution->Trees.begin(); it != it_end; it++){
			delete *it;
		}
		this->Last_solution->Trees.clear();

	}

	void I_Planner::Set_Solution(const I_Planner::__last_solution_info& last_sol) {

		if (this->Last_solution == nullptr)
			this->Last_solution = new __last_solution_info();
		else
			this->__clean_trees();

		this->Last_solution->Iteration_done = last_sol.Iteration_done;
		copy_solution(this->Last_solution->Solution , last_sol.Solution);
		this->Last_solution->Trees = last_sol.Trees;

	}

	size_t	I_Planner::Get_Iteration_done() {

		if (this->Last_solution == nullptr) return 0;
		else  return this->Last_solution->Iteration_done;

	}

	void I_Planner::Get_solution(std::list<Node_State>* result) {

		if (this->Last_solution != nullptr)
			copy_solution(*result, this->Last_solution->Solution);
		else
			result->clear();

	}

	string	I_Planner::Get_Trees_as_JSON() {

		string JSON;
		if (this->Last_solution != nullptr) {
			JSON += "[\n";
			if (!this->Last_solution->Trees.empty()) {
				JSON += "{\"Tree\":";
				auto it = this->Last_solution->Trees.begin();
				JSON += (*it)->Get_Tree_as_JSON();
				JSON += "}\n";
				it++;
				auto it_end = this->Last_solution->Trees.end();
				while (it != it_end) {
					JSON += ",{\"Tree\":";
					JSON += (*it)->Get_Tree_as_JSON();
					JSON += "}\n";
					it++;
				}
			}
			JSON += "]";
		}
		return move(JSON);

	}

	string	I_Planner::Get_Solution_as_JSON() {

		string JSON;
		if (this->Last_solution != nullptr) {
			size_t State_size = this->Handler->Get_State_size();


			JSON += "[\n";
			if (this->Last_solution->Solution.empty()) {
				JSON += "]";
				return move(JSON);
			}
			else if (this->Last_solution->Solution.size() == 1)
				JSON += json_parser::load_JSON(&this->Last_solution->Solution.front().operator[](0), State_size);
			else {
				auto it = this->Last_solution->Solution.begin();
				size_t K = this->Last_solution->Solution.size() - 1;
				for (size_t k = 0; k < K; k++) {
					JSON += json_parser::load_JSON(&it->operator[](0), State_size);
					JSON += ",\n";
					it++;
				}
				JSON += json_parser::load_JSON(&it->operator[](0), State_size);
				JSON += "\n";
			}
			JSON += "]";
		}
		return move(JSON);

	}

	void	I_Planner::RRT_basic(const Node_State& start, const Node_State& end) {

		this->_RRT_basic(start, end);

	}

	void	I_Planner::RRT_bidirectional(const Node_State& start, const Node_State& end) {

		if (!this->Handler->Get_symm_flag()) throw 0;
		this->_RRT_bidirectional(start, end);

	}

	void	I_Planner::RRT_star(const Node_State& start, const Node_State& end) {

		bool temp = this->Cumulate_sol;
		this->Cumulate_sol = true;
		this->_RRT_star(start, end);
		this->Cumulate_sol = temp;

	}

}