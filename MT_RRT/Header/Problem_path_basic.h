/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#pragma once
#ifndef __MT_RRT_PATH_BASIC_H__
#define __MT_RRT_PATH_BASIC_H__

#include "Problem_description.h"
#include <vector>

namespace MT_RTT
{

	/** \brief Interface for handling classical path planning problem of a single or a group of articulated fixed robots (Section 2.2 of the documentation).
	\details The steering configurations lie always along a segment in the configurational space connecting
	the nearest neighbour to the target pose.
	When considering multi-robot problems, the state Q of a single node in a tree is a buffer containing the poses of all the robots in a single array  as follows:
	Q = [Q1^T, Q2^T, ..., Qn^T], where Qi is the pose of the i-th robot.
	*/
	class Manipulator_path_handler : public Linear_traj_factory {
	protected:
		/** \brief The hypercube delimited by the maximal and minimum possible joint excursions is sampled.
		*/
		virtual void									Random_node(float* random_state);

		/** \brief Constructor. 
		\details Q_min and Q_max are the joint limits, i.e. any pose must be Q_min < Q < Q_max (for each joint).
		A consistency check is internally done to ensure that for each joint i: Q_max[i] >= Q_min[i].
		In case multiple robots are part of the cell, each Q_max and Q_min are buffers composed as follows:
		Q_max = [Q_max1^T, Q_max2^T, ..., Q_maxn^T], Q_min = [Q_min1^T, Q_min2^T, ..., Q_minn^T]
		* @param[in] Gamma the parameter regulating the near set size (see Node::I_Node_factory::Get_Gamma)
		* @param[in] Q_max the maximal values allowed for each joint of the robot(s)
		* @param[in] Q_min the minimum values allowed for each joint of the robot(s)
		*/
		Manipulator_path_handler(const float& Gamma, const Array& Q_max, const Array& Q_min, const float& steer_degree);

		const Array&									Get_max() const { return this->Max_Q_vals; };
		const Array&									Get_min() const { return this->Min_Q_vals; };
	private:
	// data
		Array				Max_Q_vals;
		Array				Min_Q_vals;
		Array				Delta_Q_vals;
	};


	/** \brief In this object, the collision along a certain segment in the configurational space (i.e. a trajectory connecting two nodes) is
	checked considering a discrete set of equispaced samples, Section 2.2.3 of the documentation.
	\details An external object, in charge of performing the collision check must be passed and absorbed.
	*/
	class Tunneled_check_collision : public Manipulator_path_handler { //the passed I_Collision_checker is absorbed and destroyed when this object is destroyed
	public:
		/** \brief The object in charge of performing the collision check for a single generic configuration. 
		*/
		class I_Collision_checker {
		public:
			virtual ~I_Collision_checker(){};

			/** \brief It is mainly used when copying the Tunneled_check_collision object that contains this checker.
			\details All the parameters inside the object must be copied, since the copied object will be used by a different thread.
			* @param[out] return a copy of this object inside a smart pointer
			*/
			virtual std::unique_ptr<I_Collision_checker>	copy_checker() = 0;

			/** \brief Returns true when a collision is detected for the configuration passed as input
			* @param[out] return the result of the collision check detection
			* @param[in] Q_state the pose to check
			*/			
			virtual bool									Collision_present(const float* Q_state) = 0;
		};
		/** \brief Returns the checker contained in this object.
		*/
		const I_Collision_checker*								Get_checker() const { return this->Collision_checker.get(); };

		/** \brief Constructor.
		* @param[in] coll_checker the object in charge of performing the collision check of a single pose
		* @param[in] Gamma same meaning as in Manipulator_path_handler::Manipulator_path_handler
		* @param[in] steer_degree a steered pose lies in the segment connecting the nearest neighbour to the target node, at a 
		distance (euclidean distance in the configurational space) which is at most equal to the steer_degree
		* @param[in] Q_max same meaning as in Manipulator_path_handler::Manipulator_path_handler
		* @param[in] Q_min same meaning as in Manipulator_path_handler::Manipulator_path_handler
		*/					
		Tunneled_check_collision(const float& Gamma, const float& steer_degree, const Array& Q_max, const Array& Q_min, std::unique_ptr<I_Collision_checker>& coll_checker);

		/** \brief Constructor.
		\details Similar to Tunneled_check_collision::Tunneled_check_collision(const float& Gamma, const float& steer_degree, const Node_State& Q_max, const Node_State& Q_min, std::unique_ptr<I_Collision_checker>& coll_checker),
		but assuming that Q_max (and Q_min) have all the same values equal to q_max and has a size equal to dof.
		*/
		Tunneled_check_collision(const float& Gamma, const float& steer_degree, const float& q_max, const float& q_min, const size_t& dof, std::unique_ptr<I_Collision_checker>& coll_checker);

		virtual std::unique_ptr<I_Node_factory>			copy() { return std::unique_ptr<I_Node_factory>(new Tunneled_check_collision(*this)); };
		
	private:
		Tunneled_check_collision(Tunneled_check_collision& o);

		virtual bool									Check_reached_in_cache();
	// data
		std::unique_ptr<I_Collision_checker>			 Collision_checker;
	};



	/** \brief In this object, the steering is done considering bubbles of free configuration, Section 2.2.3 of the documentation.
	\details An external object, in charge of computing the distance between the robot(s) links and the obstacles is passed and absorbed.
	*/
	class Bubbles_free_configuration : public Manipulator_path_handler { //the passed I_Proximity_calculator is absorbed and destroyed when this object is destroyed
	public:
		/** \brief The object in charge of computing the distances between the robots and the obstacles, which are used for computing the bubbles.
		*/
		class I_Proximity_calculator {
		public:
			virtual ~I_Proximity_calculator(){ delete this->Robot_distance_pairs; };

			/** \brief It is mainly used when copying the Bubbles_free_configuration object that contains this calculator.
			\details All the parameters inside the object must be copied, since the copied object will be used by a different thread.
			* @param[out] return a copy of this object
			*/
			virtual std::unique_ptr<I_Proximity_calculator>			copy_calculator() = 0;
			
			/** \brief The information about the distances between the robots and the obstacles are recomputed considering
			the new passed pose.
			* @param[in] Q_state the new pose to consider for updating the distances values
			*/			
			virtual void											Recompute_Proximity_Info(const float* Q_state) = 0;

			/** \brief The distances concerning a single robot
			*/			
			struct single_robot_prox {
				float											Distance_to_fixed_obstacles; //the minimum distance d^i from robot i to all the obstacles, Section 2.2.3 of the documentation.
				Array								            Radii; //radius {r^i1, r^i2, ... } to compute for determine a bubble of free configurations, Section 2.2.3 of the documentation.

				single_robot_prox(const size_t& dof): Radii(0.f , dof) {};
			};
			
			/** \brief Get the last computed distances w.r.t the obstacles and robot
			*/			
			const std::vector<single_robot_prox>&					Get_single_info() const { return this->Robots_info; };

			/** \brief Get the last reciprocal computed distances between the robots
			*/						
			const Array*								            Get_distances_pairs() const { return this->Robot_distance_pairs; };			
		protected:
			I_Proximity_calculator(const std::vector<size_t>& Dof);
		// data
			std::vector<single_robot_prox>						Robots_info;
			Array*											    Robot_distance_pairs; //distance d^ik between the robots, Section 2.2.3 of the documentation. It can be NULL when a single robot populates the scene.
		};
		const I_Proximity_calculator*									   Get_proxier() const { return this->Proximity_calculator.get(); };

		/** \brief Constructor. The passed prox_calc is absorbed and destroyed when destroying this object
		* @param[in] prox_calc the object in charge of computing the distances w.r.t to the obstacles and the reciprocal distances of the robots
		* @param[in] Gamma same meaning as in Manipulator_path_handler::Manipulator_path_handler
		* @param[in] Q_max same meaning as in Manipulator_path_handler::Manipulator_path_handler
		* @param[in] Q_min same meaning as in Manipulator_path_handler::Manipulator_path_handler
		*/	
		Bubbles_free_configuration(const float& Gamma, const Array& Q_max, const Array& Q_min, std::unique_ptr<I_Proximity_calculator>& prox_calc);

		/** \brief Constructor.
		\details Similar to Bubbles_free_configuration(const float& Gamma, const Node_State& Q_max, const Node_State& Q_min, std::unique_ptr<I_Proximity_calculator>& prox_calc),
		but assuming that Q_max (and Q_min) have all the same values equal to q_max and has a size equal to dof.
		*/
		Bubbles_free_configuration(const float& Gamma, const float& q_max, const float& q_min, const size_t& dof, std::unique_ptr<I_Proximity_calculator>& prox_calc);

		virtual std::unique_ptr<I_Node_factory>			copy() { return std::unique_ptr<I_Node_factory>(new Bubbles_free_configuration(*this)); };

		/** \brief Set the threshold for accepting a steering operation.
		\details When considering the bubble of free configuration, the steering is always possible, but can lead to obtain a configuration
		very close to the nearest neighbour (when the robots are close to the obstacles). This function regulates the threashold that is used
		to decide or not to accept a new steered configuration.
		*/	
		void												Set_dist_for_accept_steer(const float& value);
	private:
		Bubbles_free_configuration(Bubbles_free_configuration& o);

		/** \brief It implements the concept described in the Figure 2.5 of the documentation.
		*/
		class bubble_trajectory : public I_trajectory{
		public:
			bubble_trajectory(const float* start, const float* end, Bubbles_free_configuration* caller);

			virtual float 								Cost_to_go();
			virtual bool								Advance();
		};
		virtual void									Recompute_trajectory_in_cache(const float*  Start, const float*  End) { this->last_computed_traj = new bubble_trajectory(Start, End, this); };

		virtual bool									Check_reached_in_cache();
	// data
		float											Min_dist_for_accept_steer;
		std::unique_ptr<I_Proximity_calculator>         Proximity_calculator;
		bool											Force_Check_reached_in_cache;
	};

};

#endif
