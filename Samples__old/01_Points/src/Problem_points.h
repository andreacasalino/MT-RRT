/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#pragma once
#ifndef  __PROBLEM_POINTS_H__
#define __PROBLEM_POINTS_H__

#include "../../../MT_RRT/Header/Problem_description.h"
#include <list>
#include <string>


/** \brief 2D coordinate
*/
struct Point_2D {
	Point_2D(const float& X, const float& Y) : x(X), y(Y) {};
	float x;
	float y;
};

/** \brief Data type describing a box like obstacle
*/
class Box {
public:
	/** \brief Constructor. It takes two point in the space delimitating the box, i.e. the two vertices having the maximal and the minium possible x,y coordinates
	* @param[in] A the first vertex describing the box
	* @param[in] B the second vertex describing the box
	*/
	Box(const Point_2D& A, const Point_2D& B);

	/** \brief Check whether the passed segment (described by the two passed points) collides with this box.
	\details Seg_A and Seg_B are the vertices at the beginning and the ending of the segment.
	* @param[out] return true in case a collision is present. 
	* @param[in] Seg_A the first vertex describing the segement to check for the collision
	* @param[in] Seg_B the second vertex describing the segement to check for the collision
	*/
	bool collide_with_segment(const Point_2D& Seg_A, const Point_2D& Seg_B);
	
	/** \brief Check whether the passed point (described by the two passed coordinates) is contained in this box.
	* @param[out] return true in case the point is contained. 
	* @param[in] Seg_A the first vertex describing the segement to check for the collision
	* @param[in] Seg_B the second vertex describing the segement to check for the collision
	*/	
	bool collide_with_point(const float& Px, const float& Py);

	/** \brief get the minimum value along the x axis of this box
	*/
	const float& get_x_min() const { return this->x_min; };
	
	/** \brief get the maximal value along the x axis of this box
	*/
	const float& get_x_max() const { return this->x_max; };
		
	/** \brief get the minimum value along the y axis of this box
	*/
	const float& get_y_min()  const { return this->y_min; };
		
	/** \brief get the maximal value along the y axis of this box
	*/
	const float& get_y_max()  const { return this->y_max; };

	/** \brief Initializes a random set of boxes.
	\details N_boxes are randomly placed in the space and then an algorithms tries to addense
	these boxes around N_cluster clusters.
	* @param[in] N_cluster the number of clusters to consider
	* @param[in] N_boxes the number of boxes to generate
	* @param[out] boxes the random set of generated boxes
	*/
	static void Random_generation(std::list<Box>* boxes, const size_t& N_cluster, const size_t& N_boxes);
private:
	void _get_extremals(float* min, float* max, const Point_2D& Dir);
// data
	float			x_min;
	float			x_max;
	float			y_min;
	float			y_max;
};



class Problem_points : public MT_RTT::Equispaced_Node_factory {
public:
	/** \brief Constructor.
	* @param[in] boxes the obstacles to consider
	* @param[in] limit_a the first vertex of the bounding box delimitating the workspace
	* @param[in] limit_b the second vertex of the bounding box delimitating the workspace
	*/
	Problem_points(const std::list<Box>& boxes, const Point_2D& limit_a, const Point_2D& limit_b);
	
	/** \brief Constructor.
	\details It randomly generates a set of boxes by internally call Box::Random_generation.
	* @param[in] N_cluster same meaning as in Box::Random_generation
	* @param[in] N_boxes same meaning as in Box::Random_generation
	* @param[in] limit_a the first vertex of the bounding box delimitating the workspace
	* @param[in] limit_b the second vertex of the bounding box delimitating the workspace
	*/
	Problem_points(const size_t& N_cluster, const size_t& N_boxes, const Point_2D& limit_a, const Point_2D& limit_b);

	/** \brief The passed obstacles replace the old ones.
	* @param[in] boxes the new set of obstacles to consider
	*/
	void Set_boxes(const std::list<Box>& boxes) { this->Obstacles = boxes; };

	/** \brief Returns a json describing the boxes present in the scene
	*/
	std::string Get_as_JSON() const;

	virtual std::unique_ptr<I_Node_factory>			copy();
private:
	virtual void									Random_node(float* random_state);
	virtual bool									Check_reached_in_cache();
// data
	std::list<Box>									Obstacles;
	Box												Workspace;
};

#endif