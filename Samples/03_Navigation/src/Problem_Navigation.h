/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#ifndef __PROBLEM_NAVIGATION_H__
#define __PROBLEM_NAVIGATION_H__

#include "../../../MT_RRT/Header/Problem_description.h"
#include "../../../MT_RRT/Header/json.h"
#include "../../src/geometry.h"
using namespace MT_RTT;


/** \brief class used to describe planar navigation problems.
*/
class Navigator : public Equispaced_Node_factory {
public:
	/** \brief Used to describe an obstacle populating the scene
	*/
	struct Obstacle{
		float ray;
		float center[2];
	};
	/** \brief Used to describe the limits of the environment where the vehicle must navigate.
	*/
	struct Limits {
		float X_min;
		float X_max;
		float Y_min;
		float Y_max;
	};

	/** \brief Constructor of a navigation problem.
	* @param[in] lim the limits of the environment
	* @param[in] obstacles the obstacles populating the scene
	* @param[in] Ray the steering radius
	* @param[in] vehicle_width the width of the vehicle
	* @param[in] vehicle_long the length of the vehicle
	*/
	Navigator(const Limits& lim, const std::vector<Obstacle>& obstacles, const float& Ray, const float& vehicle_width, const float& vehicle_length);

	/** \brief import the environment from a json
	*/
	Navigator(const std::vector<json_parser::field>& json_content);

	/** \brief copy constructor
	*/
	Navigator(const Navigator& o);

	/** \brief returns the steering radius
	*/
	const float&									Get_Traj_ray() { return this->Traj_ray; };

	/** \brief returns true if there is a collision for the passed position
	*/
	bool exist_collision(const float* pos) { return this->Checker.exist_collision(pos); };
private:
	class circular_trajectory;

	class Cart_trajectory : public Composite_trajectory {
	public:
		/** \brief Used to build a trajectory from a starting configuration to an ending one.
		\details A configuration is completely defined knowing the x,y coordinates of the vehicle as well as its orientation (with this exact order in start and end).
		*/
		//const float* start, const float* end, I_Node_factory* caller

		Cart_trajectory(const float* start, const float* end, Navigator* caller);
	};

	class Collision_checker {
	public:
		Collision_checker(const float& W, const float& L, const std::vector<Obstacle>& obstacles);
		bool exist_collision(const float* pos);

		float get_W() const { return 2.f * this->Vehicle_width_half; };
		float get_L() const { return 2.f * this->Vehicle_long_half; };
		const std::vector<Obstacle>& get_Obstacles() const { return this->Obstacles; };
	protected:
	// data
		std::vector<Obstacle>	Obstacles;
		Segment_VS_Point		Board_checker1;
		Segment_VS_Point		Board_checker2;
		Segment					Segs[4];
		float					Vehicle_width_half;
		float					Vehicle_long_half;
	};

	virtual std::unique_ptr<I_Node_factory>			copy() { return std::unique_ptr<I_Node_factory>(new Navigator(*this)); };
	virtual void									Random_node(float* random_state);
	virtual bool									Check_reached_in_cache();
	virtual void									Recompute_trajectory_in_cache(const float* Start, const float* End);
// data
	Limits					limits;
	float					x_delta;
	float					y_delta;
	float					Traj_ray;
	Collision_checker		Checker;
};


#endif