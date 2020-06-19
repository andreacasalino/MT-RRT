/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "Problem_Navigation.h"
#include <cmath>
#include <float.h>
using namespace std;



Planar_trajectory::I_path::~I_path() { delete this->next;}

class Planar_trajectory::path_linear : public Planar_trajectory::I_path {
public:
	void					 init() {
		this->Delta[0] -= this->Start[0];
		this->Delta[1] -= this->Start[1];
		this->Length = sqrtf(this->Delta[0]* this->Delta[0] + this->Delta[1] * this->Delta[1]);
		this->angle = atan2f(this->Delta[1] , this->Delta[0]);
	};

	virtual void			 eval(float* configuration, const float& prctg) {
		configuration[0] = this->Start[0] + this->Delta[0] * prctg;
		configuration[1] = this->Start[1] + this->Delta[1] * prctg;
		configuration[2] = this->angle;
	};

	float				Start[2];
	float				Delta[2];
	float			    angle;
};

class Planar_trajectory::path_circular : public Planar_trajectory::I_path {
public:
	virtual void			 eval(float* configuration, const float& prctg) {
		configuration[2] = this->Orient_initial + this->Angle_delta * prctg;
		auto temp = this->Angle_initial + this->Angle_delta * prctg;
		configuration[0] = this->Center[0] + this->Ray * cosf(temp);
		configuration[1] = this->Center[1] + this->Ray * sinf(temp);
	};

	float				Ray;
	float				Center[2];

	float				Angle_delta;
	float				Angle_initial;
	float				Orient_initial;
};

Planar_trajectory::Planar_trajectory(const float& ray, const float& Startx, const float& Starty, const float& Start_angle,
	const float& Endx, const float& Endy, const float& End_angle) : Cost(FLT_MAX), Path_found(NULL) {

	Segment VA;
	float C_start = cosf(Start_angle), S_start = sinf(Start_angle);

	VA.V1[0] = Startx;
	VA.V1[1] = Starty;
	VA.V1[2] = 0.f;

	VA.V2[0] = Startx + C_start;
	VA.V2[1] = Starty + S_start;
	VA.V2[2] = 0.f;

	if (abs(Start_angle - End_angle) <= 0.01f) {
		Line_VS_Point checker(VA , Point3D(Endx , Endy, End_angle));
		if ((checker.get_s() >= 0.f) && (checker.Get_distance() <= 1e-2)) {
			auto p = new path_linear();
			p->Start[0] = Startx;
			p->Start[1] = Starty;
			p->Delta[0] = Endx;
			p->Delta[1] = Endy;
			p->init();
			this->Path_found = p;
			this->Cost = p->Length;
			p->next = nullptr;
		}
	}
	else {
		Segment VB;
		float C_end = cosf(End_angle), S_end = sinf(End_angle);

		VB.V1[0] = Endx;
		VB.V1[1] = Endy;
		VB.V1[2] = 0.f;

		VB.V2[0] = Endx + C_end;
		VB.V2[1] = Endy + S_end;
		VB.V2[2] = 0.f;

		Line_VS_Line solver(VA, VB);
		bool Solution_type;

		if (solver.get_are_parallel()) return;
		if (solver.get_a() >= 0.f && solver.get_b() <= 0.f) {
			float angle_A = atan2f(-S_start, -C_start);
			float angle_B = atan2f(   S_end,  C_end);
			float l = abs (ray / tanf(0.5f * (angle_B - angle_A)));

			path_linear*	p0 = new path_linear();
			path_circular*	p1 = new path_circular();
			path_linear*	p2 = new path_linear();

			p0->Start[0] = Startx;
			p0->Start[1] = Starty;
			p2->Delta[0] = Endx;
			p2->Delta[1] = Endy;

			if ((l < solver.get_a()) && (l < abs(solver.get_b()))) {
			//simple solution
				Solution_type = true;
				p0->Delta[0] = solver.Get_closest_in_shapeA()[0] - C_start * l;
				p0->Delta[1] = solver.Get_closest_in_shapeA()[1] - S_start * l;

				p2->Start[0] = solver.Get_closest_in_shapeA()[0] + C_end * l;
				p2->Start[1] = solver.Get_closest_in_shapeA()[1] + S_end * l;
			}
			else {
			//complex solution
				Solution_type = false;
				p0->Delta[0] = solver.Get_closest_in_shapeA()[0] + C_start * l;
				p0->Delta[1] = solver.Get_closest_in_shapeA()[1] + S_start * l;

				p2->Start[0] = solver.Get_closest_in_shapeA()[0] - C_end * l;
				p2->Start[1] = solver.Get_closest_in_shapeA()[1] - S_end * l;
			}
			float d = sqrtf(l * l + ray * ray);

			float coeff =  abs( d / (l * cosf(0.5f * (angle_B - angle_A))) );

			p1->Center[0] = 0.5f * (p0->Delta[0] + p2->Start[0]) - solver.Get_closest_in_shapeA()[0];
			p1->Center[0] *= coeff;
			p1->Center[0] += solver.Get_closest_in_shapeA()[0];

			p1->Center[1] = 0.5f * (p0->Delta[1] + p2->Start[1]) - solver.Get_closest_in_shapeA()[1];
			p1->Center[1] *= coeff;
			p1->Center[1] += solver.Get_closest_in_shapeA()[1];

			float A[2]; A[0] = p0->Delta[0] - p1->Center[0]; A[1] = p0->Delta[1] - p1->Center[1];
			float B[2]; B[0] = p2->Start[0] - p1->Center[0]; B[1] = p2->Start[1] - p1->Center[1];

			p1->Angle_initial = atan2f(A[1], A[0]);
			p1->Angle_delta = A[0] * B[0] + A[1] * B[1];
			p1->Angle_delta = p1->Angle_delta / (sqrtf(A[0] * A[0] + A[1] * A[1]) * sqrtf(B[0] * B[0] + B[1] * B[1]));
			p1->Angle_delta = acosf(p1->Angle_delta);
			float cross = A[0] * B[1] - A[1] * B[0]; //A cross B
			if (Solution_type) {
				if (cross < 0.f) p1->Angle_delta = -p1->Angle_delta;
			}
			else {
				p1->Angle_delta = 6.2832f - p1->Angle_delta;
				if (cross > 0.f) p1->Angle_delta = -p1->Angle_delta;
			}
			p1->Length = ray * abs(p1->Angle_delta);
			p1->Ray = ray;

			p0->init();
			p2->init();
			p1->Orient_initial = p0->angle;

			this->Path_found = p0;
			p0->next = p1;
			p1->next = p2;
			p2->next = nullptr;
			this->Cost = p0->Length + p1->Length + p2->Length;
		}
	}

}

Planar_trajectory::iterator::iterator(const Planar_trajectory* solution, const float& max_space) : attual_portion(solution->Path_found), delta_space_max(max_space), cumulated_space(0.f) {

	if (solution->Path_found == NULL) throw 0;

	this->attual_portion->eval(&this->position[0] , 0.f);
	this->__init_portion();

}

void Planar_trajectory::iterator::__init_portion() {

	this->k = 0;
	this->K = (size_t)ceilf(this->attual_portion->Length / this->delta_space_max);
	this->cumulated_delta = this->attual_portion->Length / (float)this->K;

}

bool Planar_trajectory::iterator::is_not_at_end() {  return !((this->attual_portion->next == nullptr) && (this->k == this->K)); }

Planar_trajectory::iterator& Planar_trajectory::iterator::operator++() {

	if ((this->attual_portion->next == nullptr) && (this->k == this->K)) throw 0; //iterator was already a tthe end of the path, cannot be incremented

	this->k++;
	this->cumulated_space += this->cumulated_delta;
	this->attual_portion->eval(&this->position[0], (float)(this->k + 1) / (float)this->K);
	if (this->k == this->K) {
		if (this->attual_portion->next == nullptr) return *this;
		this->attual_portion = this->attual_portion->next;
		this->__init_portion();
	}
	return *this;

}



float get_gamma(const Navigator::Limits& lim) {

	float gamma = lim.X_max - lim.X_min;
	float temp = lim.Y_max - lim.Y_min;
	if (temp > gamma) gamma = temp;
	return 0.25f * gamma;

}
Navigator::Navigator(const Limits& lim, const std::vector<Obstacle>& obstacles, const float& Ray, const float& vehicle_width, const float& vehicle_long) :
Node_factory_concrete( 3, get_gamma(lim) ,false), x_min(lim.X_min), y_min(lim.Y_min), Traj_ray(Ray), Checker(vehicle_width, vehicle_long) {

	if (lim.X_max < lim.X_min) throw 0;
	if (lim.Y_max < lim.Y_min) throw 0;
	if (this->Traj_ray < 0.f) throw 0;
	if (this->Traj_ray < 1e-2) throw 0;

	this->x_delta = lim.X_max - lim.X_min;
	this->y_delta = lim.Y_max - lim.Y_min;
	this->x_max = lim.X_max;
	this->y_max = lim.Y_max;

	this->Steer_degree = this->x_delta * 0.01f;
	float temp = this->y_delta * 0.01f;
	if (temp < this->Steer_degree) this->Steer_degree = temp;

	this->Obstacles = obstacles;

}

Navigator build_from_JSON(const std::vector<json_parser::field>& json_content) {

	vector<Navigator::Obstacle>		circles;

	auto obst = json_parser::get_field(json_content, "obstacles");
	if (obst != NULL) {
		if (!obst->front().empty()) {
			circles.reserve(obst->size());
			for (size_t o = 0; o < obst->size(); o++) {
				circles.emplace_back();
				circles.back().center[0] = (*obst)[o][0];
				circles.back().center[1] = (*obst)[o][1];
				circles.back().ray = (*obst)[o][2];
			}
		}
	}

	auto info = json_parser::get_field(json_content, "info");
	Navigator::Limits lim;
	lim.X_min = (*info)[0][0];
	lim.X_max = (*info)[0][1];
	lim.Y_min = (*info)[0][2];
	lim.Y_max = (*info)[0][3];
	float ray = (*info)[0][4];
	float Width = (*info)[0][5];
	float Long = (*info)[0][6];

	return Navigator(lim , circles, ray, Width, Long);

}
Navigator::Navigator(const std::vector<json_parser::field>& json_content) : Navigator(build_from_JSON(json_content)) {};

Navigator::Limits Navigator::__get_limits() const {
	Navigator::Limits temp;
	temp.X_min = this->x_min;
	temp.X_max = this->x_min + this->x_delta;
	temp.Y_min = this->y_min;
	temp.Y_max = this->y_min + this->y_delta;
	return temp;
}
Navigator::Navigator(const Navigator& o) : Navigator( o.__get_limits(), o.Obstacles, o.Traj_ray, o.Checker.get_W(), o.Checker.get_L()) { };

void Navigator::Random_node(float* random_state) {

	random_state[0] = this->x_min + this->x_delta * (float)rand() / (float)RAND_MAX;
	random_state[1] = this->y_min + this->y_delta * (float)rand() / (float)RAND_MAX;
	random_state[2] = -3.14159f + 6.28318f * (float)rand() / (float)RAND_MAX;

}

void Navigator::Cost_to_go(float* result, const float* start_state, const float* ending_state) {

	Planar_trajectory traj(this->Traj_ray, start_state[0], start_state[1], start_state[2],
										  ending_state[0], ending_state[1], ending_state[2]);
	*result = traj.get_cost();

}

void Navigator::Cost_to_go_constraints(float* result, const float* start_state, const float* ending_state) {

	Planar_trajectory traj(this->Traj_ray, start_state[0], start_state[1], start_state[2],
		ending_state[0], ending_state[1], ending_state[2]);

	*result = traj.get_cost();
	if (*result == FLT_MAX) return;

	Planar_trajectory::iterator it_traj(&traj, this->Steer_degree);
	++it_traj;
	const float* pos = it_traj.get_position();
	while (it_traj.is_not_at_end()) {
		if (this->Checker.exist_collision(pos, this->Obstacles) || this->__is_outside(pos)) {
			*result = FLT_MAX;
			return;
		}
		++it_traj;
	}

}

void Navigator::Steer(float* cost_steered, float* steered_state, const float* start_state, const float* target_state, bool* trg_reached) {

	Planar_trajectory traj(this->Traj_ray, start_state[0], start_state[1], start_state[2],
		target_state[0], target_state[1], target_state[2]);
	*trg_reached = false;

	*cost_steered = FLT_MAX;
	if (traj.get_cost() == FLT_MAX) return;

	Planar_trajectory::iterator it_traj(&traj, this->Steer_degree);
	++it_traj;
	const float* pos = it_traj.get_position();
	while (it_traj.is_not_at_end()) {
		if (this->Checker.exist_collision(pos, this->Obstacles) || this->__is_outside(pos)) return;
		steered_state[0] = pos[0];
		steered_state[1] = pos[1];
		steered_state[2] = pos[2];
		*cost_steered = it_traj.get_cumulated_space();
		++it_traj;
	}
	*trg_reached = true;

}

std::vector<std::vector<float>>	 Navigator::Compute_interpolated_path(const std::vector<std::vector<float>>& waypoints) {

	std::vector<std::vector<float>>	path;
	auto it_w = waypoints.begin();
	path.push_back({ (*it_w)[0],(*it_w)[1],(*it_w)[2] });
	auto it_prev = it_w;
	it_w++;
	auto it_w_end = waypoints.end();
	for (it_w = it_w; it_w != it_w_end; it_w++) {
		Planar_trajectory traj(this->Traj_ray, (*it_prev)[0], (*it_prev)[1], (*it_prev)[2],
											   (*it_w)[0], (*it_w)[1], (*it_w)[2]);
		Planar_trajectory::iterator it_traj(&traj, this->Steer_degree);
		const float* pos = it_traj.get_position();
		++it_traj;
		while (it_traj.is_not_at_end()) {
			path.push_back({ pos[0],pos[1],pos[2] });
			++it_traj;
		}
		it_prev++;
	}
	return path;

}

Segment get_segment(const float& x1, const float& y1, const float& x2, const float& y2) {
	Segment dummy;
	dummy.V1 = Point3D(x1, y1);
	dummy.V2 = Point3D(x2, y2);
	return dummy;
}
Navigator::Collision_checker::Collision_checker(const float& W, const float& L):
 Board_checker1(get_segment(1.f, 0.f, 0.f, 1.f) , Point3D( 0.f, 0.f)), Board_checker2(get_segment(1.f, 0.f, 0.f, 1.f), Point3D(0.f, 0.f)),
 Vehicle_width_half(W*0.5f), Vehicle_long_half(L * 0.5f){
	this->Segs[0].V1[0] =  this->Vehicle_long_half; this->Segs[0].V1[1] = -this->Vehicle_width_half; this->Segs[0].V2[0] =  this->Vehicle_long_half; this->Segs[0].V2[1] =  this->Vehicle_width_half;
	this->Segs[1].V1[0] =  this->Vehicle_long_half; this->Segs[1].V1[1] =  this->Vehicle_width_half; this->Segs[1].V2[0] = -this->Vehicle_long_half; this->Segs[1].V2[1] =  this->Vehicle_width_half;
	this->Segs[2].V1[0] = -this->Vehicle_long_half; this->Segs[2].V1[1] =  this->Vehicle_width_half; this->Segs[2].V2[0] = -this->Vehicle_long_half; this->Segs[2].V2[1] = -this->Vehicle_width_half;
	this->Segs[3].V1[0] = -this->Vehicle_long_half; this->Segs[3].V1[1] = -this->Vehicle_width_half; this->Segs[3].V2[0] =  this->Vehicle_long_half; this->Segs[3].V2[1] = -this->Vehicle_width_half;
}

bool Navigator::Collision_checker::exist_collision(const float* pos, const std::vector<Obstacle>& obstacles) {

	float C_angle = cosf(pos[2]);
	float S_angle = sinf(pos[2]);
	size_t K = obstacles.size();
	float P[2], Delta[2];
	for (size_t k = 0; k < K; k++) {
		Delta[0] = pos[0] - obstacles[k].center[0];
		Delta[1] = pos[1] - obstacles[k].center[1];
		P[0] = C_angle * Delta[0] + S_angle * Delta[1];
		P[1] =-S_angle * Delta[0] + C_angle * Delta[1];
		if ( (abs(P[0]) <= this->Vehicle_long_half) && (abs(P[1]) <= this->Vehicle_width_half) )  return true;
		else {
			Point3D P_temp(P[0], P[1]);
			if ((P[0] >= 0.f) && (P[1] >= 0.f)) {
				Board_checker1.Change_Pair(this->Segs[0], P_temp);
				Board_checker2.Change_Pair(this->Segs[1], P_temp);
			}
			else if ((P[0] < 0.f) && (P[1] >= 0.f)) {
				Board_checker1.Change_Pair(this->Segs[1], P_temp);
				Board_checker2.Change_Pair(this->Segs[2], P_temp);
			}
			else if ((P[0] < 0.f) && (P[1] < 0.f)) {
				Board_checker1.Change_Pair(this->Segs[2], P_temp);
				Board_checker2.Change_Pair(this->Segs[3], P_temp);
			}
			else {
				Board_checker1.Change_Pair(this->Segs[3], P_temp);
				Board_checker2.Change_Pair(this->Segs[0], P_temp);
			}
			if (Board_checker1.Get_distance() <= obstacles[k].ray) return true;
			if (Board_checker2.Get_distance() <= obstacles[k].ray) return true;
		}
	}
	return false;

}

bool Navigator::__is_outside(const float* pos) {

	if (pos[0] < this->x_min) return true;
	if (pos[0] > this->x_max) return true;
	if (pos[1] < this->y_min) return true;
	if (pos[1] > this->y_max) return true;
	return false;

}