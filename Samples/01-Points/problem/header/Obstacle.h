/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_OBSTACLE_H
#define MT_RRT_SAMPLE_OBSTACLE_H

#include <Point.h>
#include <list>
#include <vector>

namespace mt::sample {
	/** \brief Data type describing a box like obstacle
	*/
	class Obstacle {
	public:
		/** \brief Constructor. It takes two point in the space delimitating the box, i.e. the two vertices having the maximal and the minium possible x,y coordinates
		* @param[in] A the first vertex describing the box
		* @param[in] B the second vertex describing the box
		*/
		Obstacle(const geometry::Point& A, const geometry::Point& B);

		/** \brief Check whether the passed segment (described by the two passed points) collides with this box.
		\details Seg_A and Seg_B are the vertices at the beginning and the ending of the segment.
		* @param[out] return true in case a collision is present.
		* @param[in] Seg_A the first vertex describing the segement to check for the collision
		* @param[in] Seg_B the second vertex describing the segement to check for the collision
		*/
		bool collideWithSegment(const float* pointA, const float* pointB);

		/** \brief Check whether the passed point (described by the two passed coordinates) is contained in this box.
		* @param[out] return true in case the point is contained.
		* @param[in] Seg_A the first vertex describing the segement to check for the collision
		* @param[in] Seg_B the second vertex describing the segement to check for the collision
		*/
		bool collideWithPoint(const float* coordinates);

		/** \brief get the minimum value along the x axis of this box
		*/
		inline const float& getXMin() const { return this->x_min; };

		/** \brief get the maximal value along the x axis of this box
		*/
		inline const float& getXMax() const { return this->x_max; };

		/** \brief get the minimum value along the y axis of this box
		*/
		inline const float& getYMin()  const { return this->y_min; };

		/** \brief get the maximal value along the y axis of this box
		*/
		inline const float& getYMax()  const { return this->y_max; };

		/** \brief Initializes a random set of boxes.
		\details N_boxes are randomly placed in the space and then an algorithms tries to addense
		these boxes around N_cluster clusters.
		* @param[in] N_cluster the number of clusters to consider
		* @param[in] N_boxes the number of boxes to generate
		* @param[out] boxes the random set of generated boxes
		*/
		static std::vector<Obstacle> generateRandomBoxes(const size_t& N_cluster, const size_t& N_boxes);

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