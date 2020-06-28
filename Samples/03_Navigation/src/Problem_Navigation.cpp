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



class Navigator::circular_trajectory : public I_trajectory {
public:
	circular_trajectory(float* start , float* end, Navigator* caller, const float& cx, const float& cy, const float& a_delta, const float& a_initial):
	I_trajectory(start , end, caller), angle_delta(a_delta), angle_initial(a_initial) {

		this->Center[0] = cx;
		this->Center[1] = cy;

	};

	virtual float Cost_to_go() { return abs(this->angle_delta)* static_cast<Navigator*>(this->Caller)->Get_Traj_ray(); };
	bool Advance() {

		if (this->Cursor_along_traj == nullptr) {
			this->step = 0;
			float steer_degree = dynamic_cast<Equispaced_Node_factory*>(this->Caller)->Get_Steer_degree();
			this->step_max = (size_t)ceilf(this->Cost_to_go() / steer_degree);
			this->delta = this->angle_delta / (float)this->step_max;
			this->Cumulated_cost = 0.f;

			this->Cursor_along_traj = new float[3];
			this->Cursor_previous = new float[3];
			Array::Array_copy(this->Cursor_previous, this->Start, 3);

			this->angle_attual = this->angle_initial;
			this->Cursor_along_traj[2] = this->Start[2] + this->delta;
		}
		else {
			float* temp = this->Cursor_previous;
			this->Cursor_previous = this->Cursor_along_traj;
			this->Cursor_along_traj = temp;
			this->Cursor_along_traj[2] = this->Cursor_previous[2] + this->delta;
		}
		++this->step;
		this->angle_attual += this->delta;
		float ray = static_cast<Navigator*>(this->Caller)->Get_Traj_ray();
		this->Cursor_along_traj[0] = this->Center[0] + ray * cosf(this->angle_attual);
		this->Cursor_along_traj[1] = this->Center[1] + ray * sinf(this->angle_attual);
		this->Cumulated_cost += abs(this->delta) * ray;

		return (this->step < this->step_max);

	}
private:
// data
	float		Center[2];
	float		angle_delta;
	float		angle_initial;

	size_t		step;
	size_t		step_max;
	float		angle_attual;
	float		delta;
};

Navigator::Cart_trajectory::Cart_trajectory(const float* start, const float* end, Navigator* caller) :
	Composite_trajectory(start, end, caller) {

	list<float*> wayps;
	list<I_trajectory*> pieces;

/////////////////////////////// trajectory computation 
	Segment VA;
	float C_start = cosf(start[2]), S_start = sinf(start[2]);

	VA.V1[0] = start[0];
	VA.V1[1] = start[1];
	VA.V1[2] = 0.f;

	VA.V2[0] = start[0] + C_start;
	VA.V2[1] = start[1] + S_start;
	VA.V2[2] = 0.f;

	if (abs(start[2] - end[2]) <= 0.035f) {
		Line_VS_Point checker(VA, Point3D(end[0], end[1], 0.f));
		float delta_SE[2];
		delta_SE[0] = end[0] - start[0];
		delta_SE[1] = end[1] - start[1];
		if ((checker.get_s() >= 0.f) && (checker.Get_distance() <= 0.01f) && (sqrtf(delta_SE[0] * delta_SE[0] + delta_SE[1] * delta_SE[1]) >= 0.1f))  
			pieces.push_back(new linear_trajectory(start, end, dynamic_cast<Equispaced_Node_factory*>(this->Caller)));
	}
	else {
		Segment VB;
		float C_end = cosf(end[2]), S_end = sinf(end[2]);

		VB.V1[0] = end[0];
		VB.V1[1] = end[1];
		VB.V1[2] = 0.f;

		VB.V2[0] = end[0] + C_end;
		VB.V2[1] = end[1] + S_end;
		VB.V2[2] = 0.f;

		Line_VS_Line solver(VA, VB);

		if (!solver.get_are_parallel()) {
			if (solver.get_a() >= 0.f && solver.get_b() <= 0.f) {
				bool Solution_type;

				float angle_A = atan2f(-S_start, -C_start);
				float angle_B = atan2f(S_end, C_end);
				float ray = caller->Get_Traj_ray();
				float l = abs(ray / tanf(0.5f * (angle_B - angle_A)));

				float* circle_begin = new float[3];
				float* circle_end = new float[3];

				if ((l < solver.get_a()) && (l < abs(solver.get_b()))) {
					//simple solution
					Solution_type = true;
					circle_begin[0] = solver.Get_closest_in_shapeA()[0] - C_start * l;
					circle_begin[1] = solver.Get_closest_in_shapeA()[1] - S_start * l;
					circle_begin[2] = start[2];

					circle_end[0] = solver.Get_closest_in_shapeA()[0] + C_end * l;
					circle_end[1] = solver.Get_closest_in_shapeA()[1] + S_end * l;
					circle_end[2] = end[2];
				}
				else {
					//complex solution
					Solution_type = false;
					circle_begin[0] = solver.Get_closest_in_shapeA()[0] + C_start * l;
					circle_begin[1] = solver.Get_closest_in_shapeA()[1] + S_start * l;
					circle_begin[2] = start[2];

					circle_end[0] = solver.Get_closest_in_shapeA()[0] - C_end * l;
					circle_end[1] = solver.Get_closest_in_shapeA()[1] - S_end * l;
					circle_end[2] = end[2];
				}
				wayps.push_back(circle_begin);
				wayps.push_back(circle_end);

				float d = sqrtf(l * l + ray * ray);

				float coeff = abs(d / (l * cosf(0.5f * (angle_B - angle_A))));

				float center[2];
				center[0] = 0.5f * (wayps.front()[0] + wayps.back()[0]) - solver.Get_closest_in_shapeA()[0];
				center[0] *= coeff;
				center[0] += solver.Get_closest_in_shapeA()[0];

				center[1] = 0.5f * (wayps.front()[1] + wayps.back()[1]) - solver.Get_closest_in_shapeA()[1];
				center[1] *= coeff;
				center[1] += solver.Get_closest_in_shapeA()[1];

				float A[2]; A[0] = circle_begin[0] - center[0]; A[1] = circle_begin[1] - center[1];
				float B[2]; B[0] = circle_end[0] - center[0];   B[1] = circle_end[1] - center[1];

				float angle_initial;
				angle_initial = atan2f(A[1], A[0]);
				float angle_delta;
				angle_delta = A[0] * B[0] + A[1] * B[1];
				angle_delta = angle_delta / (sqrtf(A[0] * A[0] + A[1] * A[1]) * sqrtf(B[0] * B[0] + B[1] * B[1]));
				angle_delta = acosf(angle_delta);
				float cross = A[0] * B[1] - A[1] * B[0]; //A cross B
				if (Solution_type) {
					if (cross < 0.f) angle_delta = -angle_delta;
				}
				else {
					angle_delta = 6.2832f - angle_delta;
					if (cross > 0.f) angle_delta = -angle_delta;
				}

				pieces.push_back(new linear_trajectory(start, wayps.front(), dynamic_cast<Equispaced_Node_factory*>(this->Caller)));
				pieces.push_back(new circular_trajectory(wayps.front(), wayps.back(), static_cast<Navigator*>(this->Caller), center[0], center[1], angle_delta, angle_initial));
				pieces.push_back(new linear_trajectory(wayps.back(),    end, dynamic_cast<Equispaced_Node_factory*>(this->Caller)));
			}
		}
	}

	//if (force_solution && (this->Path_found == NULL)) {
	//	auto p = new path_linear();
	//	p->Start[0] = Startx;
	//	p->Start[1] = Starty;
	//	p->Delta[0] = Endx;
	//	p->Delta[1] = Endy;
	//	p->init();
	//	this->Path_found = p;
	//	this->Cost = p->Length;
	//	p->next = nullptr;
	//}
////////////////////////////////////////////////////////////

	this->Init_Pieces(pieces , wayps);

};







float get_gamma(const Navigator::Limits& lim) {

	float gamma = lim.X_max - lim.X_min;
	float temp = lim.Y_max - lim.Y_min;
	if (temp > gamma) gamma = temp;
	return 5.f * gamma;

}
float get_steer_degree(const Navigator::Limits& lim) {
	
	float temp = (lim.X_max - lim.X_min) * 0.025f;
	float temp2 = (lim.Y_max - lim.Y_min) * 0.025f;
	if (temp2 < temp) temp = temp2;
	return temp;

}
Navigator::Navigator(const Limits& lim, const std::vector<Obstacle>& obstacles, const float& Ray, const float& vehicle_width, const float& vehicle_long) :
	Equispaced_Node_factory(3, get_gamma(lim), get_steer_degree(lim), false), limits(lim), Traj_ray(Ray), Checker(vehicle_width, vehicle_long, obstacles) {

	if (lim.X_max < lim.X_min) throw 0;
	if (lim.Y_max < lim.Y_min) throw 0;
	if (this->Traj_ray < 0.f) throw 0;
	if (this->Traj_ray < 1e-2) throw 0;

	this->x_delta = lim.X_max - lim.X_min;
	this->y_delta = lim.Y_max - lim.Y_min;

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

	return Navigator(lim, circles, ray, Width, Long);

}
Navigator::Navigator(const std::vector<json_parser::field>& json_content) : Navigator(build_from_JSON(json_content)) {};

Navigator::Navigator(const Navigator& o) : Navigator(o.limits, o.Checker.get_Obstacles(), o.Traj_ray, o.Checker.get_W(), o.Checker.get_L()) { };

void Navigator::Recompute_trajectory_in_cache(const float* Start, const float* End) { this->last_computed_traj = new Cart_trajectory(Start, End, this); };

void Navigator::Random_node(float* random_state) {

	random_state[0] = this->limits.X_min + this->x_delta * (float)rand() / (float)RAND_MAX;
	random_state[1] = this->limits.Y_min + this->y_delta * (float)rand() / (float)RAND_MAX;
	random_state[2] = -3.14159f + 6.28318f * (float)rand() / (float)RAND_MAX;

}

bool Navigator::Check_reached_in_cache() { 

	const float* state = this->last_computed_traj->Get_state_current();

// check collisions
	if (this->Checker.exist_collision(state)) return true;

// check is inside region
	if (state[0] < this->limits.X_min) return true;
	if (state[0] > this->limits.X_max) return true;
	if (state[1] < this->limits.Y_min) return true;
	if (state[1] > this->limits.Y_max) return true;

	return false;

}

Segment get_segment(const float& x1, const float& y1, const float& x2, const float& y2) {
	Segment dummy;
	dummy.V1 = Point3D(x1, y1);
	dummy.V2 = Point3D(x2, y2);
	return dummy;
}
Navigator::Collision_checker::Collision_checker(const float& W, const float& L, const std::vector<Obstacle>& obstacles) :
	Obstacles(obstacles), Board_checker1(get_segment(1.f, 0.f, 0.f, 1.f), Point3D(0.f, 0.f)), Board_checker2(get_segment(1.f, 0.f, 0.f, 1.f), Point3D(0.f, 0.f)),
	Vehicle_width_half(W * 0.5f), Vehicle_long_half(L * 0.5f) {
	this->Segs[0].V1[0] = this->Vehicle_long_half; this->Segs[0].V1[1] = -this->Vehicle_width_half; this->Segs[0].V2[0] = this->Vehicle_long_half; this->Segs[0].V2[1] = this->Vehicle_width_half;
	this->Segs[1].V1[0] = this->Vehicle_long_half; this->Segs[1].V1[1] = this->Vehicle_width_half; this->Segs[1].V2[0] = -this->Vehicle_long_half; this->Segs[1].V2[1] = this->Vehicle_width_half;
	this->Segs[2].V1[0] = -this->Vehicle_long_half; this->Segs[2].V1[1] = this->Vehicle_width_half; this->Segs[2].V2[0] = -this->Vehicle_long_half; this->Segs[2].V2[1] = -this->Vehicle_width_half;
	this->Segs[3].V1[0] = -this->Vehicle_long_half; this->Segs[3].V1[1] = -this->Vehicle_width_half; this->Segs[3].V2[0] = this->Vehicle_long_half; this->Segs[3].V2[1] = -this->Vehicle_width_half;
}

bool Navigator::Collision_checker::exist_collision(const float* pos) {

	float C_angle = cosf(pos[2]);
	float S_angle = sinf(pos[2]);
	size_t K = this->Obstacles.size();
	float P[2], Delta[2];
	for (size_t k = 0; k < K; k++) {
		Delta[0] = pos[0] - this->Obstacles[k].center[0];
		Delta[1] = pos[1] - this->Obstacles[k].center[1];
		P[0] = C_angle * Delta[0] + S_angle * Delta[1];
		P[1] = -S_angle * Delta[0] + C_angle * Delta[1];
		if ((abs(P[0]) <= this->Vehicle_long_half) && (abs(P[1]) <= this->Vehicle_width_half))  return true;
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
			if (Board_checker1.Get_distance() <= this->Obstacles[k].ray) return true;
			if (Board_checker2.Get_distance() <= this->Obstacles[k].ray) return true;
		}
	}
	return false;

}
