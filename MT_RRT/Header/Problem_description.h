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
	private:
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
		virtual											~I_Node_factory() {};
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
		void											Cost_to_go(float* result, const Node* start, const Node* ending_node) { return this->Cost_to_go(result, start->Get_State(), ending_node->Get_State()); };

		/** \brief Evaluates the constrained cost of the trajectory going from the starting node to the ending one, for two nodes not already connected.
		\details This cost accounts for constraints. In case the constraints are violated along the nominal trajectory going from
		the starting node to the ending one, a FLT_MAX is returned. Otherwise, the cost returned is the one of the nominal trajectory, i.e. the
		one computed with I_Node_factory::Cost_to_go.
		* @param[out] result the computed cost
		* @param[in] start the starting node in the trajectory whose cost is to evaluate
		* @param[in] ending_node the ending node in the trajectory whose cost is to evaluate
		*/
		void											Cost_to_go_constraints(float* result, const Node* start, const  Node* trg) { return this->Cost_to_go_constraints(result, start->Get_State(), trg->Get_State()); };

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

		/** \brief Returns the cardinality of \mathcal{X}, Section 1.2.1 of the documentation, of the plannig problem handled by this object.
		*/
		virtual const size_t& Get_State_size() const = 0;

		/** \brief Returns the \gamma parameter, Section 1.2.3 of the documentation, regulating the near set size, that RRT* versions must compute.
		*/
		virtual const float& Get_Gamma() const = 0;

		/** \brief Returns true in case the planning problem handled by this object is symmetric, i.e. the cost to go from a node A to B is the same of the cost to go from B to A.
		*/
		virtual const bool& Get_symm_flag() const = 0;

		/** \brief Builds a new root for a tree.
		\details The root is a node having a NULL father.
		* @param[in] state the state that will be contained in the root to create.
		* @param[out] return the created root . 
		*/
		Node											New_root(const Array& state);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//methods to customize for the specific planning problem to address

		/** \brief used for cloning this object: a deep copy must be implemented.
		\details This function is invoked by parallel planners for dispatching copies of this class to the other working threads.
		In this way, the threads must not be forced to synchronize for accessing the methods of an I_Node_factory.
		Therefore, when deriving your own factory describing your own problem, be carefull to avoid shallow copies and implement deep copies.
		* @param[out] return a clone of this object
		*/
		virtual std::unique_ptr<I_Node_factory>			copy() = 0;

		/** \brief internally called by I_Node_factory::Random_node().
		\details The passed random_state is an array of values already with the cardinality of \mathcal{X}, which must be
		set to random values by this function.
		*/
		virtual void									Random_node(float* random_state) = 0;

		/** \brief internally called by I_Node_factory::Cost_to_go(float* result, const Node* start, const  Node* trg).
		\details This is function is actually in charge of computing C( \tau_{start -> ending_node} ), considering the passed
		start_state and ending_state, which are array of values describing the staring and the ending state to consider.
		*/
		virtual void									Cost_to_go(float* result, const float* start_state, const float* ending_state) = 0;

		/** \brief internally called by I_Node_factory::Cost_to_go_constraints(float* result, const Node* start, const  Node* trg).
		\details This function is actually in charge of computing max{  C( \tau_{start -> ending_node} )  ,  C_adm}, considering the passed
		start_state and ending_state, which are array of values describing the staring and the ending state to consider.
		*/
		virtual void									Cost_to_go_constraints(float* result, const float* start_state, const float* ending_state) = 0;

		/** \brief internally called by I_Node_factory::Steer(Node* start, const  Node* trg, bool* trg_reached).
		\details This function is actually in charge of performing the steering operation, considering the passed
		start_state and target_state, which are array of values describing the starting and target state to consider.
		cost_steered must be returned equal to NULL in case the steering was not possible.
		*/
		virtual void									Steer(float* cost_steered, float* steered_state, const float* start_state, const float* target_state, bool* trg_reached) = 0;

//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

	protected:
		float*											Alloc_state();
		
		I_Node_factory() {};
	};



	/** \brief Each handler describing a real planning problem must be derived from this class.
	*/
	class Node_factory_concrete : public Node::I_Node_factory {
	public:
		const size_t&									Get_State_size() const { return this->State_size; };
		const float&									Get_Gamma() const { return this->Gamma_coeff; };
		const bool&										Get_symm_flag() const { return this->Traj_symmetric; };
	protected:
		/** \brief Constructor of a new concrete problem
		* @param[in] X_size the cardinality of the space where the planning problem lives
		* @param[in] gamma the \gamma (see Node::I_Node_factory::Get_Gamma()) parameter used by RRT*
		* @param[in] traj_symm_flag a flag explaining whether the problem is symmetric or not, see Node::I_Node_factory::Get_symm_flag()
		*/
		Node_factory_concrete(const size_t& X_size, const float& gamma, const bool& traj_symm_flag) :
			State_size(X_size), Traj_symmetric(traj_symm_flag), Gamma_coeff(gamma) { if(X_size == 0) throw 0; };
	private:
	// data
		size_t										State_size;
		bool										Traj_symmetric;
		float										Gamma_coeff;
	};



	/** \brief Interface for the generic I_Node_factory decorator
	*/
	class I_Node_factory_decorator : public Node::I_Node_factory {
	public:
		/** \brief The I_Node_factory is absorbed and destroyed by ~I_Node_factory_decorator.
		\details Not familiar with the concept of decorator? Check https://en.wikipedia.org/wiki/Decorator_pattern.
		*/
		I_Node_factory_decorator(std::unique_ptr<I_Node_factory>& to_wrap) { this->Wrapped = move(to_wrap); };

		virtual void									Random_node(float* random_state) { this->Wrapped->Random_node(random_state); };
		virtual void									Cost_to_go(float* result, const float* start_state, const float* ending_state) { this->Wrapped->Cost_to_go(result, start_state, ending_state); };
		virtual void									Cost_to_go_constraints(float* result, const float* start_state, const float* ending_state) { this->Wrapped->Cost_to_go_constraints(result, start_state, ending_state); };
		virtual void									Steer(float* cost_steered, float* steered_state, const float* start_state, const float* target_state, bool* trg_reached) { this->Wrapped->Steer(cost_steered, steered_state, start_state, target_state, trg_reached); };

		virtual const size_t&							Get_State_size() const { return this->Wrapped->Get_State_size(); };
		virtual const float&							Get_Gamma() const { return this->Wrapped->Get_Gamma(); };
		virtual const bool&								Get_symm_flag() const { return this->Wrapped->Get_symm_flag(); };

		/** \brief 
		* @param[out] return the contained Node_factory . 
		*/
		std::unique_ptr<I_Node_factory>&				Get_Wrapped() { return this->Wrapped; };
	private:
		std::unique_ptr<I_Node_factory>					Wrapped;
	};

	

	/** \brief Used for performing each steer operation multiple times, trying to reach faster the target node.
	\details The \gamma parameter (see Node::I_Node_factory::Get_Gamma) is set equal to the one of the wrapped I_Node_factory times the number of maximal steering triala.
	*/
	class Node_factory_multiple_steer : public I_Node_factory_decorator {
	public:
		/**
		* @param[in] to_wrap the I_Node_factory to absorb
		* @param[in] max_numb_trials the maximum number of times for which the Steer must be tried 
		*/
		Node_factory_multiple_steer(std::unique_ptr<I_Node_factory>& to_wrap, const size_t& max_numb_trials);

		virtual std::unique_ptr<I_Node_factory>		copy();

		/** \brief The Steer function of the wrapped I_Node_factory is called for a maximum number of times equal to Maximum_trial in order to reach target_state.
		\details The procedure is terminated before reaching the maximum trials, when a state not compliant with the constraints is reached. 
		*/
		virtual void								Steer(float* cost_steered, float* steered_state, const float* start_state, const float* target_state, bool* trg_reached);
		virtual const float&						Get_Gamma() { return this->Gamma_multiple; };
	private:
		size_t									Maximum_trial;
		float									Gamma_multiple;
	};

}

#endif