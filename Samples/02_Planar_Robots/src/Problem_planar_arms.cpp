/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "Problem_planar_arms.h"
#include "../../src/geometry.h"
#include <cmath>
#include <float.h>
#include <list>
#include <iostream>
using namespace std;


Circle::Circle(const float& cx, const float& cy, const float& ray) : radius(ray) {
	if (ray < 0) {
		cout << "negative ray\n";
		throw 0;
	}
	this->center[0] = cx; this->center[1] = cy;
};

Robot_info::Robot_info(const float& bx, const float& by, const vector<float>& distances, const std::vector<float>& rays) :
	Robot_info(bx, by, &distances[0], &rays[0], distances.size()) { if (rays.size() != distances.size()) throw 0; };

Robot_info::Robot_info(const float& bx, const float& by, const float* distances_buffer, const float* rays_buffer, const size_t& DOF) {

	this->Base[0] = bx;
	this->Base[1] = by;

	if (DOF == 0) {
		cout << "null DOF\n";
		throw 0;
	}
	this->Link_distances.reserve(DOF);
	this->Link_Rays.reserve(DOF);
	for (size_t k = 0; k < DOF; k++) {
		if (distances_buffer[k] < 0) {
			cout << "negative distance link\n";
			throw 1;
		}
		this->Link_distances.push_back(distances_buffer[k]);
		if (rays_buffer[k] < 0) {
			cout << "negative ray link\n";
			throw 2;
		}
		this->Link_Rays.push_back(rays_buffer[k]);
	}

}




void dir_kyne(const float* Q, const Robot_info& rob, float* buffer_pos) {

	buffer_pos[0] = rob.get_base()[0];
	buffer_pos[1] = rob.get_base()[1];
	size_t k = 2;
	float q_sum = 0.f;
	auto Dist = rob.get_distances();
	size_t Kq = Dist->size();
	for (size_t kq = 0; kq < Kq; kq++) {
		q_sum += Q[kq];
		buffer_pos[k]     = buffer_pos[k - 2] + (*Dist)[kq] * cosf(q_sum);
		buffer_pos[k + 1] = buffer_pos[k - 1] + (*Dist)[kq] * sinf(q_sum);
		k += 2;
	}

}

float min_dist_chain_obstacles(const float* pos, const float* rays, const size_t& dof, const vector<Circle>& obstacles) {

	Point3D V;
	Segment S;
	float min = FLT_MAX, temp;
	size_t k;
	S.V1[0] = 1.f; S.V1[1] = 0.f; S.V1[2] = 0.f;
	S.V2[0] = 0.f; S.V2[1] = 1.f; S.V2[2] = 0.f;
	V[0] = 0.f; V[1] = 0.f; V[2] = 1.f;
	Segment_VS_Point solver(S , V);
	for (auto it = obstacles.begin(); it != obstacles.end(); it++) {
		for (k = 0; k < dof; k++) {
			S.V1[0] = pos[2 * k]; 
			S.V1[1] = pos[2 * k + 1];
			S.V2[0] = pos[2 * (k + 1)];
			S.V2[1] = pos[2 * (k + 1)+ 1];

			V[0] = it->get_center()[0];
			V[1] = it->get_center()[1];

			solver.Change_Pair(S, V);
			temp = solver.Get_distance() - rays[k] - it->get_radius();
			if (temp < min) min = temp;
		}
	}
	if (min < 0.f) min = 0.f;

	return min;

};

float min_dist_chain_chain(const float* pos1, const float* rays1, const size_t& dof1, const float* pos2, const float* rays2, const size_t& dof2) {

	Segment S1;
	S1.V1[0] = 0.f; S1.V1[1] = 0.5f; S1.V1[2] = 0.f;
	S1.V2[0] = 0.f; S1.V2[1] = 1.f; S1.V2[2] = 0.f;
	Segment S2;
	S2.V1[0] = 0.5f; S2.V1[1] = 0.f; S2.V1[2] = 0.f;
	S2.V2[0] = 1.f; S2.V2[1] = 0.f; S2.V2[2] = 0.f;
	Segment_VS_Segment solver(S1, S2);
	size_t k2;
	float min = FLT_MAX, temp;
	for (size_t k = 0; k < dof1; k++) {
		for (k2 = 0; k2 < dof2; k2++) {
			S1.V1[0] = pos1[2 * k];
			S1.V1[1] = pos1[2 * k + 1];
			S1.V2[0] = pos1[2 * (k + 1)];
			S1.V2[1] = pos1[2 * (k + 1) + 1];

			S2.V1[0] = pos2[2 * k2];
			S2.V1[1] = pos2[2 * k2 + 1];
			S2.V2[0] = pos2[2 * (k2 + 1)];
			S2.V2[1] = pos2[2 * (k2 + 1) + 1];

			solver.Change_Pair(S1, S2);
			temp = solver.Get_distance() - rays1[k] - rays2[k2];
			if (temp < min) min = temp;
		}
	}
	if (min < 0.f) min = 0.f;

	return min;

}




vector<size_t> get_dofs(const std::vector<Robot_info>& robots) {
	vector<size_t> dofs;
	auto it_end = robots.end();
	for (auto it = robots.begin(); it != it_end; it++)
		dofs.push_back(it->get_distances()->size());
	return dofs;
};
Scene_Proximity_calculator::Scene_Proximity_calculator(const std::vector<Circle>& circles, const std::vector<Robot_info>& robots) :
I_Proximity_calculator(get_dofs(robots) ){ 

	this->Obstacle = circles;
	this->Robots = robots;

	this->Joint_positions.reserve(this->Robots.size());
	size_t k, K;
	this->Joint_positions.reserve(this->Robots.size());
	for (auto it = this->Robots.begin(); it != this->Robots.end(); it++) {
		K = (it->get_distances()->size() + 1) * 2;
		this->Joint_positions.push_back(vector<float>());
		this->Joint_positions.back().reserve(K);
		for (k = 0; k < K; k++) this->Joint_positions.back().push_back(0.f);
	}

};

Scene_Proximity_calculator::Scene_Proximity_calculator(const Scene_Proximity_calculator& o) :
	I_Proximity_calculator(get_dofs(o.Robots)) {
	this->Robots = o.Robots;
	this->Obstacle = o.Obstacle;
	this->Joint_positions = o.Joint_positions;
};

Scene_Proximity_calculator build_from_JSON(const std::vector<json_parser::field>& json_content) {

	vector<Circle>		circles;
	vector<Robot_info>	robots;

	auto obst = json_parser::get_field(json_content, "obstacles");
	if (obst != NULL) {
		if (!obst->front().empty()) {
			circles.reserve(obst->size());
			for (size_t o = 0; o < obst->size(); o++)
				circles.emplace_back((*obst)[o][0], (*obst)[o][1], (*obst)[o][2]);
		}
	}

	auto rob = json_parser::get_field(json_content, "robots");
	if (rob == NULL) throw 0;
	robots.reserve(rob->size());
	for (size_t r = 0; r < rob->size(); r++) {
		const vector<float>& v = (*rob)[r];
		size_t dof = (v.size() - 2) / 2;
		robots.emplace_back(v[0], v[1], &v[2], &v[2 + dof], dof);
	}

	return Scene_Proximity_calculator(circles, robots);

}
Scene_Proximity_calculator::Scene_Proximity_calculator(const std::vector<json_parser::field>& json_content) :
	Scene_Proximity_calculator(build_from_JSON(json_content)) {};



void norm_dist(float* res, const float* P1, const float* P2) {
	*res = sqrtf( (P1[0] - P2[0])*(P1[0] - P2[0]) + (P1[1] - P2[1]) * (P1[1] - P2[1]));
};
void compute_radii(float* radii, const float* pos, const size_t& dof) {

	size_t k2;
	float temp;
	for (size_t k = 0; k < dof; k++) {
		radii[k] = 0.f;
		for (k2 = (k + 1); k2 < (dof + 1); k2++) {
			norm_dist(&temp , &pos[2 * k], &pos[2 * k2]);
			if (temp > radii[k])
				radii[k] = temp;
		}
	}

}
void	Scene_Proximity_calculator::Recompute_Proximity_Info(const float* Q_state) {

	//dir kyne and dist to obstacles and radii
	size_t kQ = 0, k;
	size_t K = this->Robots.size();
	for (k = 0; k < K; k++) {
		dir_kyne(&Q_state[kQ], this->Robots[k], &this->Joint_positions[k][0]);
		kQ += this->Robots[k].get_distances()->size();

		this->Robots_info[k].Distance_to_fixed_obstacles = min_dist_chain_obstacles(&this->Joint_positions[k][0], &(*this->Robots[k].get_rays())[0], this->Robots[k].get_distances()->size(), this->Obstacle);

		compute_radii(&this->Robots_info[k].Radii[0] , &this->Joint_positions[k][0], this->Robots[k].get_distances()->size());
	}

	// distances between robots
	size_t k_dist = 0, k2;
	for (k = 0; k < K; k++) {
		for (k2 = k + 1; k2 < K; k2++) {
			(*this->Robot_distance_pairs)[k_dist] = 
				min_dist_chain_chain(&this->Joint_positions[k][0] ,  &(*this->Robots[k].get_rays())[0], this->Robots[k].get_distances()->size(), 
									 &this->Joint_positions[k2][0], &(*this->Robots[k2].get_rays())[0], this->Robots[k2].get_distances()->size());
			k_dist++;
		}
	}

}

std::vector<size_t>	 Scene_Proximity_calculator::Get_Dofs() const {

	vector<size_t> dofs;
	size_t K = this->Robots.size();
	dofs.reserve(K);
	for (size_t k = 0; k < K; k++) 
		dofs.push_back(this->Robots[k].get_distances()->size());
	return dofs;

}

size_t Scene_Proximity_calculator::Get_Dof_tot() const {

	size_t T = 0;
	size_t K = this->Robots.size();
	for (size_t k = 0; k < K; k++)
		T += this->Robots[k].get_distances()->size();
	return T;

}




bool	Scene_Collision_checker::Collision_present(const float* Q_state) {

	this->Prox_calc.Recompute_Proximity_Info(Q_state);

	const vector<Bubbles_free_configuration::I_Proximity_calculator::single_robot_prox>& Robots_info = this->Prox_calc.Get_single_info();
	for (auto it = Robots_info.begin(); it != Robots_info.end(); it++) {
		if (it->Distance_to_fixed_obstacles == 0.f) return true;
	}

	const float* Robot_distance_pairs = &(*this->Prox_calc.Get_distances_pairs())[0];
	size_t K = this->Prox_calc.Get_distances_pairs()->size();
	for(size_t k=0; k<K; ++k){
		if(Robot_distance_pairs[k] == 0.f) return true;
	}
	return false;

}
