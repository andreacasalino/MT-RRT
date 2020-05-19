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

	float*  alloc_state_copy(const float* state, const size_t& Size) {

		float* temp = new float[Size];
		for (size_t k = 0; k < Size; k++)
			temp[k] = state[k];
		return temp;

	};

	Manipulator_path_handler::Manipulator_path_handler(const float& Gamma, const float* Q_max, const float* Q_min, const size_t& Q_size) :
		Node_factory_concrete(Q_size, Gamma, true) {

		if (Q_size == 0) throw 0;

		this->Max_Q_vals = alloc_state_copy(Q_max, Q_size);
		this->Min_Q_vals = alloc_state_copy(Q_min, Q_size);
		this->Delta_Q_vals = alloc_state_copy(Q_max, Q_size);
		for (size_t k = 0; k < Q_size; k++)
			this->Delta_Q_vals[k] -= this->Min_Q_vals[k];

		for (size_t k = 0; k < this->Get_State_size(); k++) {
			if (this->Delta_Q_vals[k] <= 0.f) throw 1;
		}

	}

	vector<float>	Equal(const float& val, const size_t& Size) {

		vector<float> Q;
		Q.reserve(Size);
		for (size_t k = 0; k < Size; k++) Q.emplace_back(val);
		return move(Q);

	}
	
	void	Manipulator_path_handler::Random_node(float* random_state) {

		auto S = this->Get_State_size();
		for (size_t k = 0; k < S; k++)
			random_state[k] = this->Min_Q_vals[k] + this->Delta_Q_vals[k] * (float)rand() / (float)RAND_MAX;

	}

	void	Manipulator_path_handler::Cost_to_go(float* result, const float* start_state, const float* ending_state) {

		*result = 0.f;
		auto S = this->Get_State_size();
		for (size_t k = 0; k < S; k++)
			*result += (ending_state[k] - start_state[k]) * (ending_state[k] - start_state[k]);
		*result = sqrtf(*result);

	}




	Tunneled_check_collision::Tunneled_check_collision(const float& Gamma, const float& steer_degree, const Node_State& Q_max, const Node_State& Q_min, unique_ptr<I_Collision_checker>& coll_checker):
		Manipulator_path_handler( Gamma, &Q_max[0], &Q_min[0], Q_max.size()), Steer_degree(steer_degree), Collision_checker(move(coll_checker)) { this->__init(); }

	Tunneled_check_collision::Tunneled_check_collision(const float& Gamma, const float& steer_degree, const float& q_max, const float& q_min, const size_t& dof, std::unique_ptr<I_Collision_checker>& coll_checker) :
		Manipulator_path_handler(Gamma, Equal(q_max, dof), Equal(q_min, dof)), Steer_degree(steer_degree), Collision_checker(move(coll_checker)) { this->__init(); };

	Tunneled_check_collision::Tunneled_check_collision(Tunneled_check_collision& o) :
		Manipulator_path_handler(o.Get_Gamma(), o.Get_max(), o.Get_min(), o.Get_State_size()), Steer_degree(o.Steer_degree), Collision_checker(move(o.Collision_checker->copy_checker())) { this->__init(); };

	void Tunneled_check_collision::__init() {

		this->__state_temp = new float[this->Get_State_size()];
		this->__delta = new float[this->Get_State_size()];

	}

	void	Tunneled_check_collision::Steer(float* cost_steered, float* steered_state, const float* start_state, const float* target_state, bool* trg_reached) {

		this->Cost_to_go(cost_steered, start_state, target_state);

		auto S = this->Get_State_size();
		if (*cost_steered <= this->Steer_degree) {
			*trg_reached = true;
			for (size_t k = 0; k < S; k++)
				steered_state[k] = target_state[k];
		}
		else {
			*trg_reached = false;
			float s = this->Steer_degree / (*cost_steered);
			*cost_steered = this->Steer_degree;
			float s2 = 1.f - s;
			for (size_t k = 0; k < S; k++) {
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
		for (k = 0; k < S; k++) {
			this->__state_temp[k] = start_state[k];
			this->__delta[k] = rec_N * (ending_state[k] - start_state[k]);
		}
		for (size_t n = 1; n < N; n++) { //n starts from 1 since the last pose should be ending_state, which is redundant to be checked
			for (k = 0; k < S; k++)
				this->__state_temp[k] += this->__delta[k];

			if (this->Collision_checker->Collision_present(this->__state_temp)) {
				*result = FLT_MAX;
				return;
			}
		}
	}




	Bubbles_free_configuration::I_Proximity_calculator::I_Proximity_calculator(const std::vector<size_t>& Dof) {

		if (Dof.empty()) throw 0;

		this->Robots_info.reserve(Dof.size());
		for (size_t k = 0; k < Dof.size(); k++) {
			this->Robots_info.emplace_back();
			this->Robots_info.back().Radii.reserve(Dof[k]);
			for (size_t k2 = 0; k2 < Dof[k]; k2++)
				this->Robots_info.back().Radii.emplace_back();
		}
		size_t N = (Dof.size() * (Dof.size() - 1)) / 2;
		this->Robot_distance_pairs.reserve(N);
		for (size_t k = 0; k < N; k++)
			this->Robot_distance_pairs.emplace_back();

	}

	Bubbles_free_configuration::Bubbles_free_configuration(const float& Gamma, const Node_State& Q_max, const Node_State& Q_min, unique_ptr<I_Proximity_calculator>& prox_calc) :
		Manipulator_path_handler(Gamma, &Q_max[0], &Q_min[0], Q_max.size()), Min_dist_for_accept_steer(DEFAULT_MIN_DIST_BUBBLE), Proximity_calculator(move(prox_calc)) { this->fake_steered = new float[this->Get_State_size()]; };

	Bubbles_free_configuration::Bubbles_free_configuration(const float& Gamma, const float& q_max, const float& q_min, const size_t& dof, std::unique_ptr<I_Proximity_calculator>& prox_calc):
		Manipulator_path_handler(Gamma, Equal(q_max, dof), Equal(q_min, dof)), Min_dist_for_accept_steer(DEFAULT_MIN_DIST_BUBBLE), Proximity_calculator(move(prox_calc)) { this->fake_steered = new float[this->Get_State_size()]; };

	Bubbles_free_configuration::Bubbles_free_configuration(Bubbles_free_configuration& o):
		Manipulator_path_handler(o.Get_Gamma(), o.Get_max(), o.Get_min(), o.Get_State_size()), Min_dist_for_accept_steer(o.Min_dist_for_accept_steer), Proximity_calculator(move(o.Proximity_calculator->copy_calculator())) { this->fake_steered = new float[this->Get_State_size()];}

	void	Bubbles_free_configuration::Set_dist_for_accept_steer(const float& value) {

		if (value < DEFAULT_MIN_DIST_BUBBLE) return;
		this->Min_dist_for_accept_steer = value;

	}

	void	Bubbles_free_configuration::Steer(float* cost_steered, float* steered_state, const float* start_state, const float* target_state, bool* trg_reached) {

		*trg_reached = false;

		this->Proximity_calculator->Recompute_Proximity_Info(start_state);
		const std::vector<I_Proximity_calculator::single_robot_prox>& info_single = this->Proximity_calculator->Get_single_info();
		const std::vector<float>&	 info_pairs = this->Proximity_calculator->Get_distances_pairs();
		vector<float> R;
		R.reserve(info_single.size());
		size_t k, K = info_single.size(), t = 0;
		auto it_single_end = info_single.end();
		for (auto it_single = info_single.begin(); it_single != it_single_end; it_single++) {
			R.emplace_back(0.f);
			K = it_single->Radii.size();
			for (k = 0; k < K; k++) {
				R.back() += it_single->Radii[k] * abs(start_state[t] - target_state[t]);
				t++;
			}
		}

		float s = 1.f, s2;
		K = R.size();
		for (k = 0; k < K; k++) {
			s2 = info_single[k].Distance_to_fixed_obstacles / R[k];
			if (s2 < s)
				s = s2;
		}
		auto it_pair = info_pairs.begin();
		for (k = 0; k < K; k++) {
			for (t = (k+1); t < K; t++) {
				s2 = *it_pair / (R[k] + R[t]);
				if (s2 < s)
					s = s2;
				it_pair++;
			}
		}

		if (s < DEFAULT_MIN_DIST_BUBBLE) {
			*cost_steered = FLT_MAX;
			return;
		}

		K = this->Get_State_size();
		if (s == 1.f) {
			*trg_reached = true;
			for (k = 0; k < K; k++)
				steered_state[k] = target_state[k];
		}
		else {
			s2 = 1.f - s;
			for (k = 0; k < K; k++)
				steered_state[k] = s2 * start_state[k] + s * target_state[k];
		}
		this->Cost_to_go(cost_steered, start_state, steered_state);

	}

	void	Bubbles_free_configuration::Cost_to_go_constraints(float* result, const float* start_state, const float* ending_state) {

		bool temp;
		this->Steer(result ,this->fake_steered, start_state,  ending_state, &temp);
		if (!temp) *result = FLT_MAX;

	}

}
