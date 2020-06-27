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

private:
	class circular_trajectory;
	class Cart_trajectory;

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



/** \brief class used to compute a trjectory from a starting state to an ending one.
*/
class Navigator::Cart_trajectory : public Node::I_Node_factory::Composite_trajectory {
public:
	/** \brief Used to build a trajectory from a starting configuration to an ending one.
	\details A configuration is completely defined knowing the x,y coordinates of the vehicle as well as its orientation.
	You can detect infeasibilities by using Planar_trajectory::get_cost.
	* @param[in] ray the radius of the vehicle steering (assumed constant)
	* @param[in] Startx x coordinate of the starting configuration
	* @param[in] Starty y coordinate of the starting configuration
	* @param[in] Start_angle orientation of the vehicle in the starting configuration
	* @param[in] Endx x coordinate of the ending configuration
	* @param[in] Endy y coordinate of the ending configuration
	* @param[in] End_angle orientation of the vehicle in the ending configuration
	*/
	//const float* start, const float* end, I_Node_factory* caller

	Cart_trajectory(const float* start, const float* end, Navigator* caller);
	//Planar_trajectory(const float& ray, , const bool& force_solution = false);

//	class I_path {
//	public:
//		I_path* next;
//		float					 Length;
//		virtual void			 eval(float* configuration, const float& prctg) = 0;
//		virtual					~I_path();
//	};
//public:

	///** \brief class used to traverse a feasible trajectory
	//*/
	//class iterator {
	//public:
	//	/** \brief Can be used only to traverse a feasible solution
	//	*/
	//	iterator(const Planar_trajectory* solution, const float& max_interspace);

	//	/** \brief Returns the actual position along the path stored internally in this iterator
	//	*/
	//	const float* get_position() const { return &this->position[0]; };

	//	/** \brief Returns the total space so far traversed along the path to traverse.
	//	*/
	//	const float& get_cumulated_space() { return this->cumulated_space; };

	//	/** \brief Used to advance along the path to traverse.
	//	\details In case this iterator is already at the end of the path an exception is throwed.
	//	*/
	//	Planar_trajectory::iterator& operator++();

	//	/** \brief Used to check whether this iterator is at the end of the corresponding path
	//	*/
	//	bool is_not_at_end();
	//private:
	//	void __init_portion();
	//	// data
	//	I_path* attual_portion;
	//	float					 delta_space_max;
	//	float					 position[3];
	//	size_t					 k;
	//	size_t					 K;
	//	float					 cumulated_delta;
	//	float					 cumulated_space;
	//};

};

#endif