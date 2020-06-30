/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../Header/Planner.h"
#include <omp.h>
#include <cmath>
#include "../Header/Planner_MT.h"
using namespace std;

namespace MT_RTT
{

	class Planner_multi_agents : public I_Planner_MT {
	public:
		Planner_multi_agents(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads, const float& reallignement_percentage)
			: I_Planner_MT(det_coeff, max_iter, handler, N_threads), Reallignement_prctg(reallignement_percentage) {};
	protected:
		virtual void					  _RRT_basic(const Array& start, const Array& end);
		virtual void			  _RRT_bidirectional(const Array& start, const Array& end) { throw 0;  };//not implementable
		virtual void					   _RRT_star(const Array& start, const Array& end);
	private:
		class Tree_master : public Tree_concrete {
		public:
			Tree_master(const Array& root_state, Node::I_Node_factory* handler, const size_t& N_threads);
			~Tree_master() { 
				for (size_t k = 0; k < this->Slaves.size(); ++k) 
					delete this->Slaves[k]; 
			};

			void			  Dispatch_roots_to_slaves();
			virtual void	  Gather_results_from_slaves(const int& th_id);//must be called from all threads

			vector<I_Tree*>   get_slaves() { return cast_to_I_Tree(this->Slaves);}
		protected:
		// data
			vector<Tree_concrete*>						Slaves;
		};

		class Tree_master_star : public Tree_master {
		public:
			Tree_master_star(const Array& root_state, Node::I_Node_factory* handler, const size_t& N_threads);
			~Tree_master_star() {
				for (auto it = this->Slaves_star.begin(); it != this->Slaves_star.end(); ++it) {
					Get_Nodes_o(*it)->clear();
					delete* it;
				}
			};

			void			 Gather_results_from_slaves(const int& th_id); //must be called from all threads
		private:
		// data
			vector<Tree_star*>									Slaves_star;
			vector<list<Tree_star::Node2Node_Traj>>				Detected_rewirds;
		};

		void	_get_solution(Tree_master* Master, const Array& end) {

			vector<Single_Extension_job> Battery_solver;
			this->Init_Single_Extension_battery(&Battery_solver, Master->get_slaves(), end);

			int Threads = (int)this->get_Threads();
			size_t Batch_size = (size_t)ceil(this->Reallignement_prctg * (float)this->Iterations_Max / (float)(Threads));
			auto seeds = random_seeds(Threads);

			bool life = true;

#pragma omp parallel \
num_threads(Threads)
			{

				int th_id = omp_get_thread_num();
				srand(seeds[th_id]);

				Single_Extension_job* Job_to_do = &Battery_solver[th_id];
				for (size_t k = 0; k < this->Iterations_Max; k += Batch_size * Threads) {
					if (!life) break;

					if (th_id == 0)
						Master->Dispatch_roots_to_slaves();

#pragma omp barrier
					Job_to_do->Extend_within_iterations(Batch_size);
#pragma omp barrier
					Master->Gather_results_from_slaves(th_id);

					if (Job_to_do->Get_solution_was_found() && (!this->Cumulate_sol)) {
#pragma omp critical
						{
							life = false;
						}
					}
#pragma omp barrier
				}
			}

			__last_solution_info info;
			info.Iteration_done = Battery_solver[0].Get_Iterations();
			Single_Extension_job::Get_best_solution(&info.Solution, Battery_solver);
			info.Trees = { Master };
			this->Set_Solution(info);
			for (int k = 0; k < Threads; ++k) Battery_solver[k].Remove_Trees();

		};

		float  Reallignement_prctg;
	};
	unique_ptr<I_Planner>							I_Planner::Get_multi_ag_parall(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads, const float& reallignement_percentage) { return unique_ptr<I_Planner>(new Planner_multi_agents(det_coeff, max_iter, handler, N_threads, reallignement_percentage)); };



	Planner_multi_agents::Tree_master::Tree_master(const Array& root_state, Node::I_Node_factory* handler, const size_t& N_threads) :
		Tree_concrete(root_state, handler, false) {

		this->Slaves.reserve(N_threads);
		this->Slaves.emplace_back(new Tree_concrete(handler, false));
		for (size_t k = 1; k < N_threads; ++k)
			this->Slaves.emplace_back(new Tree_concrete(handler, true));

	}

	void Planner_multi_agents::Tree_master::Gather_results_from_slaves(const int& th_id) {

		if (th_id == 0) {
			auto Nodes = this->Get_Nodes();
			list<Node*>::iterator it2, it2_end;
			list<Node*>* Nodes_slave;
			auto it_end = this->Slaves.end();
			for (auto it = this->Slaves.begin(); it != it_end; ++it) {
				Nodes_slave = Get_Nodes_o(*it);
				it2 = Nodes_slave->begin();
				++it2;
				it2_end = Nodes_slave->end();
				for (it2 = it2; it2 != it2_end; ++it2)
					Nodes->emplace_back(*it2);
				Nodes_slave->clear();
			}
		}

	}

	void Planner_multi_agents::Tree_master::Dispatch_roots_to_slaves() {

		Node*  new_root;
		for (auto it = this->Slaves.begin(); it != this->Slaves.end(); ++it) {
			auto Problem = this->Get_Problem_Handler();
			Node temp = Problem->Random_node();
			new_root = this->Nearest_Neighbour(&temp);
			Get_Nodes_o(*it)->emplace_back(new_root);
		}

	}

	Planner_multi_agents::Tree_master_star::Tree_master_star(const Array& root_state, Node::I_Node_factory* handler, const size_t& N_threads):
		Tree_master(root_state, handler, N_threads) {

		this->Slaves_star.reserve(N_threads);
		this->Detected_rewirds.reserve(N_threads);
		for (auto it = this->Slaves.begin(); it != this->Slaves.end(); ++it) {
			this->Slaves_star.emplace_back(new Tree_star(new Tree_concrete((*it)->Get_Problem_Handler(), false), true));
			this->Detected_rewirds.emplace_back();
		}

	}

	void round_robin_rewird_gather(std::vector<std::list<Tree_star::Node2Node_Traj>>& Rewirds) {

		std::list<std::list<Tree_star::Node2Node_Traj>*> active;
		for (size_t k = 0; k < Rewirds.size(); ++k)
			active.emplace_back(&Rewirds[k]);

		std::list<std::list<Tree_star::Node2Node_Traj>*>::iterator it_active;
		Tree_star::Node2Node_Traj* temp_trj;
		while (!active.empty()) {
			it_active = active.begin();
			while (it_active != active.end()) {
				if ((*it_active)->empty()) it_active = active.erase(it_active);
				else {
					temp_trj = &(*it_active)->front();
					temp_trj->end->Set_Father(temp_trj->start, temp_trj->cost);
					(*it_active)->pop_front();
					++it_active;
				}
			}
		}

	}
	void Planner_multi_agents::Tree_master_star::Gather_results_from_slaves(const int& th_id) {

		auto Nodes_slave = Get_Nodes_o(this->Slaves[th_id]);
		auto Nodes_slave_star = Get_Nodes_o(this->Slaves_star[th_id]);
		
		*Nodes_slave_star = *this->Get_Nodes();

		list<Tree_star::Node2Node_Traj> temp_rew;
		list<Tree_star::Node2Node_Traj>::iterator it_temp_rew, it_temp_rew_end;
		auto it = Nodes_slave->begin();
		++it;
		for (it=it; it != Nodes_slave->end(); ++it) {
			Nodes_slave_star->push_back(*it);
			this->Slaves_star[th_id]->Connect_to_best_Father_and_eval_Rewirds(&temp_rew, *it);
			it_temp_rew_end = temp_rew.end();
			for (it_temp_rew = temp_rew.begin(); it_temp_rew != it_temp_rew_end; ++it_temp_rew)
				this->Detected_rewirds[th_id].emplace_back(*it_temp_rew);
		}

#pragma omp barrier
		if (th_id == 0) {
			this->Tree_master::Gather_results_from_slaves(0);
			round_robin_rewird_gather(this->Detected_rewirds);
		}

	}



	void Planner_multi_agents::_RRT_basic(const Array& start, const Array& end) {

		auto Master = new Tree_master(start, this->Handler, this->get_Threads());
		this->_get_solution(Master, end);

	}

	void Planner_multi_agents::_RRT_star(const Array& start, const Array& end) {

		auto Master = new Tree_master_star(start, this->Handler, this->get_Threads());
		this->_get_solution(Master, end);

	}

}