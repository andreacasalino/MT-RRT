/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_RECTANGLE_H
#define MT_RRT_SAMPLE_RECTANGLE_H

#include <Point.h>
#include <list>
#include <vector>

namespace mt::sample::geometry {
	class Rectangle {
	public:
		/** @param lower corner of the box
		 *  @param upper corner of the box
		 */
		Rectangle(const geometry::Point& A, const geometry::Point& B);
		Rectangle(const Rectangle& o);

		/** @brief Check whether the segment described by the passed points collides with this box.
		 *  @param the first point of the segment to check
		 *  @param the second point of the segment to check
		 */
		bool collideWithSegment(const float* pointA, const float* pointB) const;

		/** @brief Check whether the passed point collides with this box.
		 *  @param the point to check
		 */
		bool collideWithPoint(const float* coordinates) const;

		/** @brief get the minimum value along the x axis of this box
		 */
		inline const float& getXMin() const { return this->x_min; };

		/** @brief get the maximal value along the x axis of this box
		 */
		inline const float& getXMax() const { return this->x_max; };

		/** @brief get the minimum value along the y axis of this box
		 */
		inline const float& getYMin()  const { return this->y_min; };

		/** @brief get the maximal value along the y axis of this box
		 */
		inline const float& getYMax()  const { return this->y_max; };

		/** @brief Computes a random set of boxes.
		 * N_boxes are randomly placed in the space and then an algorithms tries to addense
		 * these boxes around N_cluster clusters.
		 * @param the number of clusters to consider
		 * @param the number of boxes to generate
		 * @return set of random rectangles
		 */
		static std::vector<Rectangle> generateRandomBoxes(const size_t& N_cluster, const size_t& N_boxes);

	private:
		void getExtremal(float& val_x, float& val_y, const geometry::Point& direction) const;
	// data
		float			x_min;
		float			x_max;
		float			y_min;
		float			y_max;
	};
}

#endif