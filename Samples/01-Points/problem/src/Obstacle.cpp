/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Obstacle.h>
#include <sampler/UniformEngine.h>

namespace mt::sample {
	inline bool are_they_separate(const float& A_min, const float& A_max, const float& B_min, const float& B_max) {
		float min;
		if (A_min > B_min) min = A_min;
		else min = B_min;

		float max;
		if (A_max < B_max) max = A_max;
		else max = B_max;

		return ((max - min) < 0.f);
	}

	Obstacle::Obstacle(const geometry::Point& A, const geometry::Point& B) {
		if (A.x() < B.x()) {
			this->x_min = A.x();
			this->x_max = B.x();
		}
		else {
			this->x_min = B.x();
			this->x_max = A.x();
		}

		if (A.y() < B.y()) {
			this->y_min = A.y();
			this->y_max = B.y();
		}
		else {
			this->y_min = B.y();
			this->y_max = A.y();
		}
	}

	bool Obstacle::collideWithSegment(const float* pointA, const float* pointB) {
		float B_min, B_max;
		//x axis
		if (pointA[0] < pointB[0]) {
			B_min = pointA[0];
			B_max = pointB[0];
		}
		else {
			B_min = pointB[0];
			B_max = pointA[0];
		}
		if (are_they_separate(this->x_min, this->x_max, B_min, B_max))  return false;
		//y axis
		if (pointA[1] < pointB[1]) {
			B_min = pointA[1];
			B_max = pointB[1];
		}
		else {
			B_min = pointB[1];
			B_max = pointA[1];
		}
		if (are_they_separate(this->y_min, this->y_max, B_min, B_max)) return false;
		// Segment direction
		geometry::Point Dir(pointB[0], pointB[1]);
		Dir.x() -= pointA[0];
		Dir.y() -= pointA[1];
		float A_min, A_max;
		A_min = Dir.x() * pointA[0] + Dir.y() * pointA[1];
		A_max = Dir.x() * pointB[0] + Dir.y() * pointB[1];
		if (A_min > A_max) {
			B_min = A_max;
			A_max = A_min;
			A_min = B_min;
		}
		this->getExtremal(B_min, B_max, Dir);
		if (are_they_separate(A_min, A_max, B_min, B_max))	return false;

		// Segment normal direction
		A_max = Dir.x();
		Dir.x() = -Dir.y();
		Dir.y() = A_max;
		A_min = Dir.x() * pointA[0] + Dir.y() * pointA[1];
		this->getExtremal(B_min, B_max, Dir);

		if ((A_min < B_min) || (A_min > B_max)) return false;

		return true;
	};

	void Obstacle::getExtremal(float& min, float& max, const geometry::Point& Dir) const {
		float max_x, min_x;
		max_x = Dir.x() * this->x_max;
		min_x = Dir.x() * this->x_min;
		if (min_x > max_x) {
			min = min_x;
			min_x = max_x;
			max_x = min;
		}

		float max_y, min_y;
		max_y = Dir.y() * this->y_max;
		min_y = Dir.y() * this->y_min;
		if (min_y > max_y) {
			min = min_y;
			min_y = max_y;
			max_y = min;
		}

		min = min_x + min_y;
		max = max_x + max_y;
	}

	bool Obstacle::collideWithPoint(const float* coordinates) {
		if ((coordinates[0] > this->x_max) || (coordinates[0] < this->x_min)) return false;
		if ((coordinates[1] > this->y_max) || (coordinates[1] < this->y_min)) return false;
		return true;
	}

	const geometry::Point* Nearest_center(const std::list<geometry::Point>& Centers, const geometry::Point& point) {
		float dist_min, dist_att;
		auto Nearest = &Centers.front();
		dist_min = powf(Centers.front().x() - point.x(), 2.f) + powf(Centers.front().y() - point.y(), 2.f);
		auto it = Centers.begin();
		it++;
		for (it = it; it != Centers.end(); it++) {
			dist_att = powf(it->x() - point.x(), 2.f) + powf(it->y() - point.y(), 2.f);
			if (dist_att < dist_min) {
				dist_min = dist_att;
				Nearest = &(*it);
			}
		}
		return Nearest;
	}
	std::vector<Obstacle> Obstacle::generateRandomBoxes(const size_t& N_cl, const size_t& N_box) {
		float aggreg_coeff = 0.7f;
		sampling::UniformRandomEngine samplerPos(0.f , 1.f);
		sampling::UniformRandomEngine samplerRadius(0.01f, 0.08f);
		sampling::UniformRandomEngine samplerAngle(10.f * 3.141f / 180.f, 80.f * 3.141f / 180.f);

		size_t N_cluster = N_cl;
		if (N_cluster == 0)  N_cluster = 1;
		size_t N_boxes = N_box;
		if (N_boxes < 3)  N_boxes = 3;

		std::list<geometry::Point> Centers;
		for (size_t k = 0; k < N_cluster; k++) {
			Centers.emplace_back(samplerPos(), samplerPos());
		}

		geometry::Point Center(0.f, 0.f);
		const geometry::Point* Center_nearest = nullptr;
		float radius;
		float angle;
		std::vector<Obstacle> boxes;
		boxes.reserve(N_box);
		for (size_t k = 0; k < N_boxes; k++) {
			Center.x() = samplerPos(); Center.y() = samplerPos();
			radius = samplerRadius();
			angle = samplerAngle();

			Center_nearest = Nearest_center(Centers, Center);
			Center.x() = (1.f - aggreg_coeff) * Center.x() + aggreg_coeff * Center_nearest->x();
			Center.y() = (1.f - aggreg_coeff) * Center.y() + aggreg_coeff * Center_nearest->y();

			boxes.emplace_back(geometry::Point(radius * cosf(angle) + Center.x(), radius * sinf(angle) + Center.y()),
							   geometry::Point(-radius * cosf(angle) + Center.x(), -radius * sinf(angle) + Center.y()));
		}
		return boxes;
	}
}