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


/** \brief class used to compute a trjectory from a starting state to an ending one. 
*/
class Planar_trajectory {
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
	Planar_trajectory(const float& ray, const float& Startx, const float& Starty, const float& Start_angle,
		const float& Endx, const float& Endy, const float& End_angle, const bool& force_solution = false);
	~Planar_trajectory() { delete this->Path_found; };

	/** \brief return the lenght of the cost.
	\details In case the involved starting state and ending one cannot be connected, a FLT_MAX is returned by this function.
	*/
	const float& get_cost() { return this->Cost; };


private:
	class I_path {
	public:
		I_path*					 next;
		float					 Length;
		virtual void			 eval(float* configuration, const float& prctg) = 0;
		virtual					~I_path();
	};
public:
	/** \brief class used to traverse a feasible trajectory
	*/
	class iterator {
	public:
		/** \brief Can be used only to traverse a feasible solution
		*/
		iterator(const Planar_trajectory* solution, const float& max_interspace);

		/** \brief Returns the actual position along the path stored internally in this iterator
		*/
		const float* get_position() const { return &this->position[0]; };

		/** \brief Returns the total space so far traversed along the path to traverse.
		*/
		const float& get_cumulated_space() { return this->cumulated_space; };

		/** \brief Used to advance along the path to traverse.
		\details In case this iterator is already at the end of the path an exception is throwed.
		*/
		Planar_trajectory::iterator& operator++();

		/** \brief Used to check whether this iterator is at the end of the corresponding path
		*/
		bool is_not_at_end();
	private:
		void __init_portion();
	// data
		I_path*					 attual_portion;
		float					 delta_space_max;
		float					 position[3];
		size_t					 k;
		size_t					 K;
		float					 cumulated_delta;
		float					 cumulated_space;
	};
private:
	class path_linear;
	class path_circular;
// data
	float						 Cost;
	I_path*						 Path_found;
};



/** \brief class used to describe planar navigation problems.
*/
class Navigator : public Node_factory_concrete {
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

	virtual std::unique_ptr<I_Node_factory>			copy() { return std::unique_ptr<I_Node_factory>(new Navigator(*this)); };
	virtual void									Random_node(float* random_state);
	virtual void									Cost_to_go(float* result, const float* start_state, const float* ending_state);
	virtual void									Cost_to_go_constraints(float* result, const float* start_state, const float* ending_state);
	virtual void									Steer(float* cost_steered, float* steered_state, const float* start_state, const float* target_state, bool* trg_reached);

	/** \brief returns the steering radius
	*/
	const float&									Get_Traj_ray() { return this->Traj_ray; };

	/** \brief interpolates the waypoints returned from an RRT solver.
	\details Internally, several Planar_trajectory::iterator objects are built to traverse the path going from a waypoint to another, in order to obtain several intermediate coordinates.
	* @param[in] waypoints the waypoints coordinates are taken from the list of Node_State returned from an RRT solver
	*/
	std::vector<std::vector<float>>					Compute_interpolated_path(const std::vector<std::vector<float>>& waypoints);
private:
	bool											__is_outside(const float* pos);
	Limits											__get_limits() const;

	class Collision_checker {
	public:
		Collision_checker(const float& W, const float& L);
		bool exist_collision(const float* pos, const std::vector<Obstacle>& obstacles);

		float get_W() const { return 2.f * this->Vehicle_width_half; };
		float get_L() const { return 2.f * this->Vehicle_long_half; };
	protected:
	// data
		Segment_VS_Point		Board_checker1;
		Segment_VS_Point		Board_checker2;
		Segment					Segs[4];
		float					Vehicle_width_half;
		float					Vehicle_long_half;
	};
// data
	float					x_min;
	float					x_delta;
	float					x_max;
	float					y_min;
	float					y_delta;
	float					y_max;
	std::vector<Obstacle>	Obstacles;

	float					Traj_ray;
	float					Steer_degree;
	Collision_checker		Checker;
};

#endif