/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#pragma once
#ifndef __MT_RRT_PLANNER_MT_H__
#define __MT_RRT_PLANNER_MT_H__

#include "Extensions.h"
#include "Planner.h"

namespace MT_RTT
{

	/** \brief This class contains common functionalities that every multi-threaded planner may use.
	*/
	class I_Planner_MT :public I_Planner {
	public:
		/** 
		* @param[out] return the number of threads considered by this multi-threaded solver
		*/	
		const size_t& get_Threads() { return this->Thread_numbers; };

		/**
		* @param[in] Threads set the number of threads to use for solving future problems.
		*/
		void		  set_Threads(const size_t& Threads);
	protected:
		I_Planner_MT(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads);
		
		/** \brief This method initializes a battery of single tree extenders (see Single_Extension_job).
		\details Each of the element in the battery will be manipulated by a single thread. Therefore, the elements in the 
		battery will be concurrently extended.
		*/
		void Init_Single_Extension_battery(std::vector<Single_Extension_job>* battery , const std::vector<I_Tree*>&  T, const Array& target);

		/** \brief This method initializes a battery of bidirectional extenders (see Bidirectional_Extension_job).
		\details Each of the element in the battery will be manipulated by a single thread. Therefore, the elements in the 
		battery will be concurrently extended.
		*/
		void Init_Bidirectional_Extension_battery(std::vector<Bidirectional_Extension_job>* battery, const std::vector<I_Tree*>&  A, const std::vector<I_Tree*>&  B);

		template<typename T>
		static std::vector<I_Tree*> cast_to_I_Tree(const std::vector<T*>& to_cast) {

			std::vector<I_Tree*> temp;
			temp.reserve(to_cast.size());
			auto it_end = to_cast.end();
			for (auto it = to_cast.begin(); it != it_end; ++it)
				temp.emplace_back(*it);
			return temp;

		};

		/** \brief Get seeds for initialize rand in a different way in all the used threads.
		\details Seeds are initialized considering the actual time.
		* @param[out] return the computed seeds 
		* @param[in]  N_seeds the number of seeds to compute 
		*/
		std::vector<unsigned int> random_seeds(const size_t& N_seeds);

	private:
		size_t			Thread_numbers;
	};

}

#endif
