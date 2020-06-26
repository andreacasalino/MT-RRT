/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#pragma once
#ifndef  __MT_RRT_PROBLEM_H__
#define __MT_RRT_PROBLEM_H__

#include <cstdlib>
#include <memory>

namespace MT_RTT
{

	/** \brief Used for storing an invariant size array of numbers.
	\details It cannot be empty.
	*/
	class Array{
	public:
		/** \brief The values in vals are copied and stored in Array::pbuffer.
		* @param[in] vals a pointer to the array of numbers describing the state to represent
		* @param[in] size the number of values in vals
		*/
		Array(const float* vals, const size_t& size);

		/** \brief val_to_repeat is copied multiple times Array::pbuffer to create an array the specified size.
		* @param[in] val_to_repeat the value to repeat in the buffer to create
		* @param[in] size the size of the buffer to create
		*/
		Array(const float& val_to_repeat, const size_t& size);

		/** \brief An array with the passed size is created, but the values are not initialized.
		\details You can use the operator[] to initialize the values one by one after creating the object.
		* @param[in] size the size of the buffer to create
		*/
		Array(const size_t& size);

		/** \brief Variadic constructor accepting a variable number of floats.
		* @param[in] args the variable number of floats to use for initializing this object
		*/
		template<typename ... Args>
		static Array build_from_numbers(Args ... args){
			size_t p_temp, Size = 0;
			auto bf = __init_variadic_buffer(Size , p_temp, args...);
			return Array(bf , Size);
		};

		Array(const Array& o);

		Array& operator=(const Array& o);

		~Array() { delete[] this->pbuffer; };

		Array() = delete;

		/** \brief Access the value at position equal to pos.
		* @param[in] pos the position of the value to acess in Array::pbuffer
		* @param[out] return the value at pos position
		*/
		float& operator[](const size_t& pos);

		/** \brief Access the value at position equal to pos.
		* @param[in] pos the position of the value to acess in Array::pbuffer
		* @param[out] return the value at pos position
		*/
		const float& operator[](const size_t& pos) const;

		/** \brief Returns the size of the represented state.
		*/
		const size_t& size()const { return this->Size; };

		/** \brief Used for cloning buffer of raw numbers. 
		 * \details destination_buffer must be already allocated with the correct size
		*/
		static void Array_copy(float* destination_buffer , const float* origin_buffer, const size_t& size);
	private:
		template<typename ... Args>
		static float* __init_variadic_buffer(size_t& Size, size_t& pos, const float& val, Args ... args){
			++Size;
			auto bf = __init_variadic_buffer(Size, pos, args...);
			--pos;
			bf[pos] = val;
			return bf;
		};
		static float* __init_variadic_buffer(size_t& Size, size_t& pos, const float& val){
			++Size;
			float* pbuffer = new float[Size];
			pos = Size - 1;
			pbuffer[pos] = val;
			return pbuffer;
		};

	// data
		float* 		pbuffer;
		size_t      Size;
	};



	/** \brief Used internally by a tree (see Tree.h) for representing a state  x \in \underline{\mathcal{X}}, Section 1.2.1 of the documentation.
	*/
	class Node {
	public:
		virtual ~Node() { delete[] this->State;  };

		/** \brief Moving constructor.
		* \details State of o is put to NULL, before trasferring the pointer to the Node to build.
		*/
		Node(Node&& o);

		Node() = delete;
		Node(const Node&) = delete;
		Node& operator=(const Node&) = delete;
		Node& operator=(Node&& o) = delete;

		/** \brief Computes the cost to go from the root to this node.
		* @param[out] result the cost to go to compute.
		*/
		void													Cost_to_root(float* result) const;
		/** \brief Similar to Node::Cost_to_root(float* result), but throwing an exception when the length of the path that must be followed to reach the root
		is higher than I_max.
		\details This method is called by Tree_star to detect the existance of loop in the tree. It is never used when _REW_DEBUG (check Tree.h) is not activated
		*/
		void													Cost_to_root(float* result, const size_t& I_max) const;

		/** \brief Computes the cost to go from the father of this node to this node.
		* @param[out] return the cost to go to return.
		*/
		const float&											Get_Cost_from_father() const { return this->Cost_traj_from_father; };

		/** \brief Returns the state represented by this node.
		* @param[out] return the address of the first value of the array representing the state.
		*/
		const float*											Get_State() const { return this->State; };
		
		/** \brief Returns the father of this node.
		\details Each node has a single father and can be the father of many nodes. 
		* @param[out] return the father node of this node.
		*/
		Node*													Get_Father() const { return this->Father; };

		/** \brief Connect this node to the new one passed as input.
		\details Each node has a single father and can be the father of many nodes. 
		* @param[in] new_father the node to assume as new father
		* @param[in] cost_from_father the cost to go from the new father
		*/
		void													Set_Father(Node* new_father, const float& cost_from_father) { this->Father = new_father; this->Cost_traj_from_father = cost_from_father; };

		class I_Node_factory;
	protected:
		Node(Node* father, const float& cost, float* state) : Father(father), Cost_traj_from_father(cost), State(state) {};
		
		Node(float* state) : Node(nullptr, 0.f, state){};
		
	// data
		Node*							Father;
		float							Cost_traj_from_father;
		float*							State;
	};



	/** \brief Interface for the class describing the particular planning problem to solve. 
	\details It is crucial for addressing step A of the pipeline presented in Section 1.3 of the documentation.
	*/
	class Node::I_Node_factory {
	public:
		virtual											~I_Node_factory() { delete this->last_computed_traj; };
		I_Node_factory(const I_Node_factory& o) = delete;
		I_Node_factory& operator=(const I_Node_factory& o) = delete;

		/** \brief Returns a node having a state randomly sampled in the \mathcal{X} space, Section 1.2.1 of the documentation.
		\details This function is invoked for randomly growing a searching tree.
		* @param[out] return the random node computed . 
		*/
		Node											Random_node();

		/** \brief Evaluates the cost C(\tau), Section 1.2.3 of the documentation, of the trajectory \tau going from the starting node to the ending one, for two nodes not already connected.
		\details This cost doesn't account for constraints, but considers only the optimal unconstrained trajectory \tau leading from the starting to the ending node.
		* @param[out] result the computed cost
		* @param[in] start the starting node in the trajectory whose cost is to evaluate
		* @param[in] ending_node the ending node in the trajectory whose cost is to evaluate
		*/
		void											Cost_to_go(float* result, const Node* start, const Node* ending_node);

		/** \brief Evaluates the constrained cost of the trajectory going from the starting node to the ending one, for two nodes not already connected.
		\details This cost accounts for constraints. In case the constraints are violated along the nominal trajectory going from
		the starting node to the ending one, a FLT_MAX is returned. Otherwise, the cost returned is the one of the nominal trajectory, i.e. the
		one computed with I_Node_factory::Cost_to_go.
		* @param[out] result the computed cost
		* @param[in] start the starting node in the trajectory whose cost is to evaluate
		* @param[in] ending_node the ending node in the trajectory whose cost is to evaluate
		*/
		void											Cost_to_go_constraints(float* result, const Node* start, const  Node* trg);

		/** \brief Performs a steering operation, Section 1.2.1 of the documentation, from a staring node to a target one.
		\details The node returned contains the steered state. In case a steering operation is not possible, a Node with a NULL State is returned.
		* @param[out] return the node with the steered configuration. 
		* @param[in] start the starting node from which the steer operation must be tried
		* @param[in] trg the target node to which the steer operation must be tried
		* @param[out] trg_reached returns true in case the steering was possible and led to reach the target node. Otherwise false is returned.
		*/
		Node											Steer(Node* start, const  Node* trg, bool* trg_reached);

		//same father is assumed
		/** \brief Generates a node with the same state and father of the node passed as input.
		* @param[out] return the cloned node. 
		* @param[in] o the node to clone
		*/
		Node											Clone_Node(const Node& o);

		/** \brief Builds a new root for a tree.
		\details The root is a node having a NULL father.
		* @param[in] state the state that will be contained in the root to create.
		* @param[out] return the created root . 
		*/
		Node											New_root(const Array& state);

		/** \brief Returns the cardinality of \mathcal{X}, Section 1.2.1 of the documentation, of the plannig problem handled by this object.
		*/
		const size_t& 									Get_State_size() const { return this->State_size; };

		/** \brief Returns the \gamma parameter, Section 1.2.3 of the documentation, regulating the near set size, that RRT* versions must compute.
		*/
		float 											Get_Gamma() const { return this->Gamma_coeff * this->Steer_max_iteration; };

		/** \brief Returns true in case the planning problem handled by this object is symmetric, i.e. the cost to go from a node A to B is the same of the cost to go from B to A.
		*/
		const bool& 									Get_symm_flag() const { return this->Traj_symmetric; };

		/** \brief Set the maximal number of iterations that are considered when doing a steering operation, Figure 2.3 of the documentation (even if the concept is applied to any kind of problem).
		\details By default this value is assumed equal to 1: a single point alogn the nominal trajectory is verified to be in the admitted region and possibly assumed as steered configuration.
		When a greater value is set, multiple steering operations are tried, in order to get as much as possible close to the target configuration.
		* @param[in] Iter_number the number of iterations to consider when doing the steering process. Pass 0 to set this value to infinity: the steering is tried till reaching the target or a configuration 
		outside of the admitted region.
		*/
		void 											Set_Steer_iterations(const size_t& Iter_number) { this->Steer_max_iteration = Iter_number; };

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// To customize for the specific planning problem to solve

		/** \brief used for cloning this object: a deep copy must be implemented.
		\details This function is invoked by parallel planners for dispatching copies of this class to the other working threads.
		In this way, the threads must not be forced to synchronize for accessing the methods of an I_Node_factory.
		Therefore, when deriving your own factory describing your own problem, be carefull to avoid shallow copies and implement deep copies.
		* @param[out] return a clone of this object
		*/
		virtual std::unique_ptr<I_Node_factory>			copy() = 0;

	protected:
		/** \brief internally called by I_Node_factory::Random_node().
		\details The passed random_state is an array of values already with the cardinality of \mathcal{X}, which must be
		set to random values by this function.
		*/
		virtual void									Random_node(float* random_state) = 0;

		class I_trajectory;
		virtual void									Recompute_trajectory_in_cache(const float*  Start, const float*  End) = 0; //put the created trajectory in last_computed_traj

		virtual bool									Check_reached_in_cache() = 0; //return true when the rached state in last_computed_traj is not valid

//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		
		
		/** \brief This oject is used to represent an optimal trajectory \tau (Section 1.2 of the documentation), going from two states \in \mathcal{X}
		\details It used to compute the distance from that states and to move along the trajectory when doing steering operations.
		*/
		class I_trajectory{
		public:
			virtual 									~I_trajectory(){ delete this->Cursor_along_traj; delete this->Cursor_previous; };

			/** \brief Returns the Cost to go of the configurations connected by this trajectory.
			\details In case a trajecctory connecting the peers of this trajectory does not exist, a FLT_MAX is returned.
			*/
			virtual float 								Cost_to_go() = 0;

			/** \brief Move along the trajectory.
			\details An internal state (gettable using Get_state_current) is set equal to the starting configuration after building this object.
			Then, by calling this method, the state is modified, advancing along the trajectory of a certain quantity.
			The old state is internally stored and is gettable using Get_state_previous.
			* @param[out] return false when the end of the trajectory is reached after calling this method (after that you cannot call again Advance).
			*/
			virtual bool								Advance() = 0;

			/** \brief See I_trajectory::Advance.
			*/
			const float*  								Get_state_current() {  return this->Cursor_along_traj; };
			/** \brief See I_trajectory::Advance.
			*/
			const float*  								Get_state_previous() { return this->Cursor_previous; };

			/** \brief See I_trajectory::Advance. Returns the cost to go from the beginning of the trajectory till the state internally reached at present.
			*/
			float&										Cost_to_go_Cumulated() { return this->Cumulated_cost; };
		protected:
			I_trajectory(const float* start, const float* end, I_Node_factory* caller) : 
			Caller(caller) , Start(start), End(end), Cursor_along_traj(nullptr), Cursor_previous(nullptr), Cumulated_cost(0.f) {};
		// data
			I_Node_factory* Caller;

			const float*  Start;
			const float*  End;
			
			float*		  Cursor_along_traj;
			float*		  Cursor_previous;
			float		  Cumulated_cost;
		};

		/** \brief Constructor of a new problem
		* @param[in] X_size the cardinality of the space where the planning problem lives
		* @param[in] gamma the \gamma (see Node::I_Node_factory::Get_Gamma()) parameter used by RRT*
		* @param[in] traj_symm_flag a flag explaining whether the problem is symmetric or not, see Node::I_Node_factory::Get_symm_flag()
		*/
		I_Node_factory(const size_t& X_size, const float& gamma, const bool& traj_symm_flag) :
			State_size(X_size), Traj_symmetric(traj_symm_flag), Gamma_coeff(gamma), Steer_max_iteration(1), last_computed_traj(nullptr) { if(X_size == 0) throw 0; };

	private:
		float*											Alloc_state();

	// data
		size_t										State_size;
		bool										Traj_symmetric;
		float										Gamma_coeff;
		size_t										Steer_max_iteration;
	protected:
	// cache
		I_trajectory*								last_computed_traj;
	};



	/** \brief It describes any kind of problem where the optimal trajectory connecting two states is simply a segment in the configurational space, like the one described in 2.2.3.1
	*/
	class Linear_traj_factory : public Node::I_Node_factory {
	protected:
		/** \brief steer_degree is the value reported as \epsilon in the Figure 2.3 of the documentation
		*/
		Linear_traj_factory(const size_t& X_size, const float& gamma, const float& steer_degree) :  I_Node_factory(X_size, gamma, true), Steer_degree(steer_degree){ if(steer_degree <= 0.f) throw 0; };

		class linear_trajectory : public I_trajectory{
		public:
			~linear_trajectory(){ delete this->Delta; };
			linear_trajectory(const float* start, const float* end, Linear_traj_factory* caller) : I_trajectory(start, end, caller), Delta(nullptr) {};

			virtual float 								Cost_to_go();
			virtual bool								Advance();

			static float								Euclidean_distance(const float* p1, const float* p2, const size_t& Size);
		protected:
			float*										Delta;
			float										Delta_norm;
			size_t										step;
			size_t										step_max;
		};
		virtual void									Recompute_trajectory_in_cache(const float*  Start, const float*  End) { this->last_computed_traj = new linear_trajectory(Start, End, this); };

		const float&									Get_Steer_degree() { return this->Steer_degree; };
	private:
	// data
		float											Steer_degree;
	};

}

#endif