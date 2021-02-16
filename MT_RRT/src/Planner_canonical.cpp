/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../Header/Planner.h"
#include "../Header/Extensions.h"
#include <ctime>
using namespace std;

namespace MT_RTT
{

	class Planner_canonical : public I_Planner {
	public:
		Planner_canonical(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler) : I_Planner(det_coeff, max_iter, handler) {};
	protected:
		virtual void					  _RRT_basic(const Array& start, const Array& end);
		virtual void		_RRT_bidirectional(const Array& start, const Array& end);
		virtual void						 _RRT_star(const Array& start, const Array& end);
	private:
		template<typename Solver>
		void						_get_solution(Solver* solver) {
#ifndef DETER_SEED
			srand((unsigned int)time(NULL));
#endif
			this->Handler->set_rand_state((unsigned int)rand());

			solver->Extend_within_iterations(this->Iterations_Max);

			__last_solution_info info;
			info.Iteration_done = solver->Get_Iterations();
			solver->Get_best_solution(&info.Solution);
			info.Trees = solver->Remove_Trees();
			this->Set_Solution(info);

		};
	};
	unique_ptr<I_Planner>	 I_Planner::Get_canonical(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler) { return unique_ptr < I_Planner>(new Planner_canonical(det_coeff, max_iter, handler)); };

	void	 Planner_canonical::_RRT_basic(const Array& start, const Array& end) {

		Tree_concrete* T = new Tree_concrete(start, this->Handler, false);
		Single_Extension_job solver(T, end, &this->Deterministic_coefficient, &this->Cumulate_sol);
		this->_get_solution(&solver);

	}

	void	 Planner_canonical::_RRT_star(const Array& start, const Array& end) {

		Tree_star* T = new Tree_star(new Tree_concrete(start, this->Handler, false), true);
		Single_Extension_job solver(T, end, &this->Deterministic_coefficient, &this->Cumulate_sol);
		this->_get_solution(&solver);

	}

	void Planner_canonical::_RRT_bidirectional(const Array& start, const Array& end) {

		Tree_concrete* T1 = new Tree_concrete(start, this->Handler, false);
		Tree_concrete* T2 = new Tree_concrete(end, this->Handler, false);
		Bidirectional_Extension_job solver(T1, T2, &this->Deterministic_coefficient, &this->Cumulate_sol);
		this->_get_solution(&solver);

	}

}