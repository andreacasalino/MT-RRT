/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#include "Problem_points.h"
#include <ctime>
#include <cmath>
#include <float.h>
using namespace std;
using namespace MT_RTT;



bool are_they_separate(const float& A_min, const float& A_max, const float& B_min, const float& B_max) {

	float min;
	if (A_min > B_min) min = A_min;
	else min = B_min;

	float max;
	if (A_max < B_max) max = A_max;
	else max = B_max;


	return ((max - min) < 0.f);

}

Box::Box(const Point_2D& A, const Point_2D& B) {

	if (A.x < B.x) {
		this->x_min = A.x;
		this->x_max = B.x;
	}
	else {
		this->x_min = B.x;
		this->x_max = A.x;
	}

	if (A.y < B.y) {
		this->y_min = A.y;
		this->y_max = B.y;
	}
	else {
		this->y_min = B.y;
		this->y_max = A.y;
	}

}

bool Box::collide_with_segment(const Point_2D& Seg_A, const Point_2D& Seg_B) {

	float B_min, B_max;

	//x axis
	if (Seg_A.x < Seg_B.x) {
		B_min = Seg_A.x;
		B_max = Seg_B.x;
	}
	else {
		B_min = Seg_B.x;
		B_max = Seg_A.x;
	}
	if (are_they_separate(this->x_min, this->x_max, B_min, B_max))  return false;

	//y axis
	if (Seg_A.y < Seg_B.y) {
		B_min = Seg_A.y;
		B_max = Seg_B.y;
	}
	else {
		B_min = Seg_B.y;
		B_max = Seg_A.y;
	}
	if (are_they_separate(this->y_min, this->y_max, B_min, B_max)) return false;

	// Segment direction
	Point_2D Dir(Seg_B);
	Dir.x -= Seg_A.x;
	Dir.y -= Seg_A.y;
	float A_min, A_max;
	A_min = Dir.x*Seg_A.x + Dir.y*Seg_A.y;
	A_max = Dir.x*Seg_B.x + Dir.y*Seg_B.y;
	if (A_min > A_max) {
		B_min = A_max;
		A_max = A_min;
		A_min = B_min;
	}
	this->_get_extremals(&B_min, &B_max, Dir);
	if (are_they_separate(A_min, A_max, B_min, B_max))	return false;

	// Segment normal direction
	A_max = Dir.x;
	Dir.x = -Dir.y;
	Dir.y = A_max;
	A_min = Dir.x*Seg_A.x + Dir.y*Seg_A.y;
	this->_get_extremals(&B_min, &B_max, Dir);


	if ((A_min < B_min) || (A_min > B_max)) return false;

	return true;

};

void Box::_get_extremals(float* min, float* max, const Point_2D& Dir) {

	float max_x, min_x;
	max_x = Dir.x*this->x_max;
	min_x = Dir.x*this->x_min;
	if (min_x > max_x) {
		*min = min_x;
		min_x = max_x;
		max_x = *min;
	}

	float max_y, min_y;
	max_y = Dir.y*this->y_max;
	min_y = Dir.y*this->y_min;
	if (min_y > max_y) {
		*min = min_y;
		min_y = max_y;
		max_y = *min;
	}

	*min = min_x + min_y;
	*max = max_x + max_y;

}

bool Box::collide_with_point(const float& Px, const float& Py) {

	if ((Px > this->x_max) || (Px < this->x_min)) return false;
	if ((Py > this->y_max) || (Py < this->y_min)) return false;
	return true;

}

float rand_Uniform(const float& min = 0.f, const float Max = 1.f) { return (Max - min) * ((float)rand() / (float)RAND_MAX) + min; };
const Point_2D* Nearest_center(const list<Point_2D>& Centers, const Point_2D& point) {

	float dist_min, dist_att;
	auto Nearest = &Centers.front();
	dist_min = powf(Centers.front().x - point.x, 2.f) + powf(Centers.front().y - point.y, 2.f);
	auto it = Centers.begin();
	it++;
	for (it = it; it != Centers.end(); it++) {
		dist_att = powf(it->x - point.x, 2.f) + powf(it->y - point.y, 2.f);
		if (dist_att < dist_min) {
			dist_min = dist_att;
			Nearest = &(*it);
		}
	}
	return Nearest;

}
void Box::Random_generation(std::list<Box>* boxes, const size_t& N_cl, const size_t& N_box) {

	srand((unsigned int)time(0));
	float aggreg_coeff = 0.7f;

	size_t N_cluster = N_cl;
	if (N_cluster == 0)  N_cluster = 1;
	size_t N_boxes = N_box;
	if (N_boxes < 3)  N_boxes = 3;

	list<Point_2D> Centers;
	for (size_t k = 0; k < N_cluster; k++)
		Centers.push_back(Point_2D(rand_Uniform(), rand_Uniform()));

	Point_2D Center(0.f, 0.f);
	const Point_2D* Center_nearest = NULL;
	float radius;
	float angle;
	for (size_t k = 0; k < N_boxes; k++) {
		Center.x = rand_Uniform(); Center.y = rand_Uniform();
		radius = rand_Uniform(0.01f, 0.08f);
		angle = rand_Uniform(10.f, 80.f) * 3.141f / 180.f;

		Center_nearest = Nearest_center(Centers, Center);
		Center.x = (1.f - aggreg_coeff)*Center.x + aggreg_coeff * Center_nearest->x;
		Center.y = (1.f - aggreg_coeff)*Center.y + aggreg_coeff * Center_nearest->y;

		boxes->push_back(Box(
			Point_2D(radius*cosf(angle) + Center.x, radius*sinf(angle) + Center.y),
			Point_2D(-radius * cosf(angle) + Center.x, -radius * sinf(angle) + Center.y)));
	}

}





float get_distance(const Point_2D& limit_a, const Point_2D& limit_b) {
	float dx = limit_a.x - limit_b.x;
	float dy = limit_a.y - limit_b.y;
	return sqrtf(dx*dx + dy * dy);
};

Problem_points::Problem_points( const std::list<Box>& boxes, const Point_2D& limit_a, const Point_2D& limit_b) :
	Linear_traj_factory( 2, get_distance(limit_a, limit_b)*10.f, get_distance(limit_a, limit_b) / 20.f) , Workspace(limit_a, limit_b) {

	this->Obstacles = boxes;

}

list<Box> get_random_boxes(const size_t& N_cluster, const size_t& N_boxes) {

	list<Box> temp;
	Box::Random_generation(&temp, N_cluster, N_boxes);
	return temp;

};
Problem_points::Problem_points(const size_t& N_cluster, const size_t& N_boxes, const Point_2D& limit_a, const Point_2D& limit_b) :
	Problem_points(get_random_boxes(N_cluster, N_boxes), limit_a, limit_b) { };

void Problem_points::Random_node(float* state) {

	state[0] = (this->Workspace.get_x_max() - this->Workspace.get_x_min()) * ((float)rand() / (float)RAND_MAX) + this->Workspace.get_x_min() ;
	state[1] = (this->Workspace.get_y_max() - this->Workspace.get_y_min()) * ((float)rand() / (float)RAND_MAX) + this->Workspace.get_y_min();

}

bool Problem_points::Check_reached_in_cache(){

	Point_2D Seg_A(this->last_computed_traj->Get_state_previous()[0], this->last_computed_traj->Get_state_previous()[1]);
	Point_2D Seg_B(this->last_computed_traj->Get_state_current()[0], this->last_computed_traj->Get_state_current()[1]);
	for (auto it = this->Obstacles.begin(); it != this->Obstacles.end(); ++it) {
		if (it->collide_with_segment(Seg_A, Seg_B)) return true;
	}
	return false;

}

std::unique_ptr<Node::I_Node_factory>		Problem_points::copy() {

	Point_2D A(this->Workspace.get_x_min(), this->Workspace.get_y_min());
	Point_2D B(this->Workspace.get_x_max(), this->Workspace.get_y_max());

	return unique_ptr<Node::I_Node_factory>(new Problem_points(this->Obstacles, A, B));

}

void append_box(std::string* JSON, const Box& b) {

	*JSON += "[";
	*JSON += to_string(b.get_x_min()) + "," + to_string(b.get_y_min()) + ",";
	*JSON += to_string(b.get_x_max()) + "," + to_string(b.get_y_max());
	*JSON += "]";

};
std::string Problem_points::Get_as_JSON() const {

	string JSON;
	JSON += "[\n";
	if (this->Obstacles.empty()) {
		JSON += "]";
		return JSON;
	}
	else if (this->Obstacles.size() == 1)
		append_box(&JSON, this->Obstacles.front());
	else  {
		auto it = this->Obstacles.begin();
		size_t K = this->Obstacles.size() -1;
		for (size_t k = 0; k < K; k++) {
			append_box(&JSON, *it);
			JSON += ",\n";
			it++;
		}
		append_box(&JSON, *it);
		JSON += "\n";
	}

	JSON += "]";
	return JSON;

}