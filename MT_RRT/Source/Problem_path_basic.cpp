/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../Header/Problem_path_basic.h"
#include <random>
#include <float.h>
using namespace std;

#define DEFAULT_MIN_DIST_BUBBLE 0.001f


namespace MT_RTT
{

	Manipulator_path_handler::Manipulator_path_handler(const float& Gamma, const Array& Q_max, const Array& Q_min) :
		Node_factory_concrete(Q_max.size(), Gamma, true), Max_Q_vals(Q_max), Min_Q_vals(Q_min), Delta_Q_vals(Q_max) {

		if(Q_max.size() != Q_min.size()) throw 0;

		for (size_t k = 0; k < Q_min.size(); ++k) this->Delta_Q_vals[k] -= this->Min_Q_vals[k];
		for (size_t k = 0; k < this->Get_State_size(); ++k) {
			if (this->Delta_Q_vals[k] <= 0.f) throw 1;
		}

	}
	
	void	Manipulator_path_handler::Random_node(float* random_state) {

		auto S = this->Get_State_size();
		for (size_t k = 0; k < S; ++k)
			random_state[k] = this->Min_Q_vals[k] + this->Delta_Q_vals[k] * (float)rand() / (float)RAND_MAX;

	}

	void	Manipulator_path_handler::Cost_to_go(float* result, const float* start_state, const float* ending_state) {

		*result = 0.f;
		auto S = this->Get_State_size();
		for (size_t k = 0; k < S; ++k)
			*result += (ending_state[k] - start_state[k]) * (ending_state[k] - start_state[k]);
		*result = sqrtf(*result);

	}




	Tunneled_check_collision::Tunneled_check_collision(const float& Gamma, const float& steer_degree, const Array& Q_max, const Array& Q_min, unique_ptr<I_Collision_checker>& coll_checker):
		Manipulator_path_handler( Gamma, Q_max, Q_min), Steer_degree(steer_degree), Collision_checker(move(coll_checker)), __state_temp(Q_max), __delta(Q_max) {}

	Tunneled_check_collision::Tunneled_check_collision(const float& Gamma, const float& steer_degree, const float& q_max, const float& q_min, const size_t& dof, std::unique_ptr<I_Collision_checker>& coll_checker) :
		Manipulator_path_handler(Gamma, Array(q_max, dof), Array(q_min, dof)), Steer_degree(steer_degree), Collision_checker(move(coll_checker)), __state_temp(q_max, dof), __delta(q_max, dof)  {};

	Tunneled_check_collision::Tunneled_check_collision(Tunneled_check_collision& o) :
		Manipulator_path_handler(o.Get_Gamma(), o.Get_max(), o.Get_min()), Steer_degree(o.Steer_degree), Collision_checker(move(o.Collision_checker->copy_checker())), __state_temp(o.Get_max()), __delta(o.Get_max()) {};

	void	Tunneled_check_collision::Steer(float* cost_steered, float* steered_state, const float* start_state, const float* target_state, bool* trg_reached) {

		this->Cost_to_go(cost_steered, start_state, target_state);

		auto S = this->Get_State_size();
		if (*cost_steered <= this->Steer_degree) {
			*trg_reached = true;
			for (size_t k = 0; k < S; ++k)
				steered_state[k] = target_state[k];
		}
		else {
			*trg_reached = false;
			float s = this->Steer_degree / (*cost_steered);
			*cost_steered = this->Steer_degree;
			float s2 = 1.f - s;
			for (size_t k = 0; k < S; ++k) {
				steered_state[k] = s * target_state[k];
				steered_state[k] += s2 * start_state[k];
			}
			if (this->Collision_checker->Collision_present(steered_state)) *cost_steered = FLT_MAX;
		}

	}

	void	Tunneled_check_collision::Cost_to_go_constraints(float* result, const float* start_state, const float* ending_state) {

		this->Cost_to_go(result, start_state, ending_state);

		if (*result < this->Steer_degree) return;

		auto S = this->Get_State_size();
		size_t N = (size_t)ceilf(*result / this->Steer_degree), k;
		float rec_N = 1.f / (float)N;
		for (k = 0; k < S; ++k) {
			this->__state_temp[k] = start_state[k];
			this->__delta[k] = rec_N * (ending_state[k] - start_state[k]);
		}
		for (size_t n = 1; n < N; ++n) { //n starts from 1 since the last pose should be ending_state, which is redundant to be checked
			for (k = 0; k < S; ++k)
				this->__state_temp[k] += this->__delta[k];

			if (this->Collision_checker->Collision_present(&this->__state_temp[0])) {
				*result = FLT_MAX;
				return;
			}
		}
	}



	Bubbles_free_configuration::I_Proximity_calculator::I_Proximity_calculator(const std::vector<size_t>& Dof) : Robot_distance_pairs(nullptr) {

		if (Dof.empty()) throw 0;

		this->Robots_info.reserve(Dof.size());
		for (size_t k = 0; k < Dof.size(); ++k)  this->Robots_info.emplace_back(Dof[k]);
		if(Dof.size() > 1) this->Robot_distance_pairs = new Array(0.f , (Dof.size() * (Dof.size() - 1)) / 2);

	}

	Bubbles_free_configuration::Bubbles_free_configuration(const float& Gamma, const Array& Q_max, const Array& Q_min, unique_ptr<I_Proximity_calculator>& prox_calc) :
		Manipulator_path_handler(Gamma, Q_max, Q_min), Min_dist_for_accept_steer(DEFAULT_MIN_DIST_BUBBLE), Proximity_calculator(move(prox_calc)), fake_steered(Q_max) {};

	Bubbles_free_configuration::Bubbles_free_configuration(const float& Gamma, const float& q_max, const float& q_min, const size_t& dof, std::unique_ptr<I_Proximity_calculator>& prox_calc):
		Manipulator_path_handler(Gamma, Array(q_max, dof), Array(q_min, dof)), Min_dist_for_accept_steer(DEFAULT_MIN_DIST_BUBBLE), Proximity_calculator(move(prox_calc)), fake_steered(q_max, dof) {};

	Bubbles_free_configuration::Bubbles_free_configuration(Bubbles_free_configuration& o):
		Manipulator_path_handler(o.Get_Gamma(), o.Get_max(), o.Get_min()), Min_dist_for_accept_steer(o.Min_dist_for_accept_steer), Proximity_calculator(move(o.Proximity_calculator->copy_calculator())), fake_steered(o.fake_steered) {}

	void	Bubbles_free_configuration::Set_dist_for_accept_steer(const float& value) {

		if (value < DEFAULT_MIN_DIST_BUBBLE) return;
		this->Min_dist_for_accept_steer = value;

	}

	void	Bubbles_free_configuration::Steer(float* cost_steered, float* steered_state, const float* start_state, const float* target_state, bool* trg_reached) {

		*trg_reached = false;

		this->Proximity_calculator->Recompute_Proximity_Info(start_state);
		const std::vector<I_Proximity_calculator::single_robot_prox>& info_single = this->Proximity_calculator->Get_single_info();
		const Array*	 info_pairs = this->Proximity_calculator->Get_distances_pairs();
		vector<float> R;
		R.reserve(info_single.size());
		size_t k, K = info_single.size(), t = 0;
		auto it_single_end = info_single.end();
		for (auto it_single = info_single.begin(); it_single != it_single_end; ++it_single) {
			R.emplace_back(0.f);
			K = it_single->Radii.size();
			for (k = 0; k < K; ++k) {
				R.back() += it_single->Radii[k] * abs(start_state[t] - target_state[t]);
				++t;
			}
		}

		float s = 1.f, s2;
		K = R.size();
		for (k = 0; k < K;++k) {
			s2 = info_single[k].Distance_to_fixed_obstacles / R[k];
			if (s2 < s)
				s = s2;
		}
		if(info_pairs != nullptr){
			const float* pair_bf = &(*info_pairs)[0];
			size_t pp = 0;
			for (k = 0; k < K; ++k) {
				for (t = (k+1); t < K; ++t) {
					s2 = pair_bf[pp] / (R[k] + R[t]);
					if (s2 < s)
						s = s2;
					++pp;
				}
			}
		}

		if (s < DEFAULT_MIN_DIST_BUBBLE) {
			*cost_steered = FLT_MAX;
			return;
		}

		K = this->Get_State_size();
		if (s == 1.f) {
			*trg_reached = true;
			for (k = 0; k < K; ++k)
				steered_state[k] = target_state[k];
		}
		else {
			s2 = 1.f - s;
			for (k = 0; k < K; ++k)
				steered_state[k] = s2 * start_state[k] + s * target_state[k];
		}
		this->Cost_to_go(cost_steered, start_state, steered_state);

	}

	void	Bubbles_free_configuration::Cost_to_go_constraints(float* result, const float* start_state, const float* ending_state) {

		bool temp;
		this->Steer(result , &this->fake_steered[0], start_state,  ending_state, &temp);
		if (!temp) *result = FLT_MAX;

	}

}
