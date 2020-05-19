/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#pragma once
#ifndef  __PROBLEM_PLANAR_ARMS_H__
#define __PROBLEM_PLANAR_ARMS_H__

#include "../../../MT_RRT/Header/Problem_path_basic.h"
#include "../../../MT_RRT/Header/json.h"
#include <string>
using namespace MT_RTT;

/** \brief class describing an obstacle with a planar circle shape
*/
class Circle {
public:
	Circle(const float& cx, const float& cy, const float& ray);

	const float& get_radius() const { return this->radius; };
	const float* get_center() const { return &this->center[0]; };
private:
	float		radius;
	float		center[2]; //x, y
};

/** \brief class describing a planar robot.
\details Link_distances are the lengths of the links, while
the Base is the center of the first joint
*/
class Robot_info {
public:
	Robot_info(const float& bx, const float& by, const std::vector<float>& distances, const std::vector<float>& rays);
	Robot_info(const float& bx, const float& by, const float* distances_buffer, const float* rays_buffer, const size_t& DOF);

	const std::vector<float>* get_distances() const { return &this->Link_distances; };
	const std::vector<float>* get_rays() const { return &this->Link_Rays; };
	const float*		      get_base() const { return &this->Base[0]; };
private:							
	std::vector<float>				Link_distances;
	std::vector<float>				Link_Rays;
	float							Base[2]; //x, y
};



/** \brief Computes the minimum distance between each robot and the circles, as well as the reciprocal distances of the robots.
*/	
class Scene_Proximity_calculator : public Bubbles_free_configuration::I_Proximity_calculator {
public:
	/** \brief Constructor. 
	* @param[in] circles the obstacles populating the scene
	* @param[in] robots the robots populating the scene
	*/	
	Scene_Proximity_calculator(const std::vector<Circle>& circles, const std::vector<Robot_info>& robots);

	Scene_Proximity_calculator(const std::vector<json_parser::field>& json_content);

	/** \brief The copy constructor. It is used by copy_calculator to clone the object
	*/	
	Scene_Proximity_calculator(const Scene_Proximity_calculator& o);

	/** \brief Refer to I_Proximity_calculator::copy_calculator
	*/
	virtual std::unique_ptr<I_Proximity_calculator>	    copy_calculator() { return std::unique_ptr<I_Proximity_calculator>(new Scene_Proximity_calculator(this->Obstacle, this->Robots)); };

	/** \brief Refer to I_Proximity_calculator::Recompute_Proximity_Info
	*/	
	virtual void										Recompute_Proximity_Info(const float* Q_state);

	/** \brief Returns the degree of freedom (dof) of every robot present in the scene
	*/	
	std::vector<size_t>									Get_Dofs() const;
	size_t												Get_Dof_tot() const;
private:
// data
	std::vector<Robot_info>								Robots;
	std::vector<Circle>									Obstacle;

	std::vector<std::vector<float>>						Joint_positions;
};

/** \brief The collision check is done internally performing the proximity computations: if all
the distances computed are greater than 0, no collisions are present.
*/
class Scene_Collision_checker : public Tunneled_check_collision::I_Collision_checker {
public:
	/** \brief Constructor. 
	* @param[in] circles the obstacles populating the scene
	* @param[in] robots the robots populating the scene
	*/	
	Scene_Collision_checker(const std::vector<Circle>& circles, const std::vector<Robot_info>& robots) :
		Prox_calc(circles, robots) {};

	Scene_Collision_checker(const std::vector<json_parser::field>& json_content) : Prox_calc(json_content) {};

	/** \brief Refer to I_Collision_checker::copy_checker
	*/
	virtual std::unique_ptr<I_Collision_checker>		copy_checker() { return std::unique_ptr<I_Collision_checker>(new Scene_Collision_checker(*this)); };

	/** \brief Refer to I_Collision_checker::Collision_present
	*/	
	virtual bool										Collision_present(const float* Q_state);

	/** \brief Access the underlying proximity query calculator
	*/	
	const Scene_Proximity_calculator*					Get_wrapped_prox() { return &this->Prox_calc; };
private:
// data
	Scene_Proximity_calculator							Prox_calc;
};

#endif