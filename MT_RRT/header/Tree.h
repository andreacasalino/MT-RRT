/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#pragma once
#ifndef __MT_RRT_TREE_H__
#define __MT_RRT_TREE_H__

#include "Problem_description.h"
#include <list>
#include <string>

namespace MT_RTT
{
	/** \brief Interface for handling a tree involved in a RRT strategy.
	*/
	class I_Tree {
	public:
		virtual ~I_Tree() {  };
		I_Tree(const I_Tree& o) = delete;
		void operator=(const I_Tree& o) = delete;

		/** \brief Tries to extend the tree toward a random configuration that is internally sampled.
		\detials In case an extensio was possible, the node added to the tree is returned. Otherwise, NULL is returned
		* @param[out] return the node added to the tree as a consequence of the extension (NULL is returned in case the extension was not possible).
		*/
		const Node*										Extend_random();

		/** \brief An extension toward the passed target node is tried.
		\details In case the extension succeeds, a new node with the steered configuration, Section 1.2 of the documentation,  is
		automatically inserted to the list of nodes contained in this tree and returned. On the opposite,
		when the extension was not possible a NULL value is returned.
		* @param[in] target the target node toward which the extension must be tried
		* @param[out] return the node added to the tree as a consequence of the extension (NULL is returned in case the extension was not possible).
		*/
		const Node*										Extend_deterministic(const Node* target);

		/** \brief Create a json describing the nodes contained in this tree.
		\details The json produced is an array of node, each having an array describing the state of the i^th node
		and an array describing the state of its parent.
		* @param[out] return the json structure describing the tree. 
		*/
		std::string										Get_Tree_as_JSON();

		/** \brief Get the object describing the planning problems this tree refers to.
		* @param[out] return the object describing the planning problem
		*/
		virtual Node::I_Node_factory*					Get_Problem_Handler() = 0;

		/** \brief Get the root of the tree.
		*/
		const Node*										Get_root() { return this->Get_Nodes()->front(); };

		/** \brief Get the reached target flag.
		\details The reach target flag describes whether the previous extension tried (using Extend_random or Extend_deterministic) suceeded in reaching the
		target or not. More formally, when a kind of extension is tried for the tree, this quantity is internally set equal to true only in the case that:
		A) the extension was possible 
		B) the steered configuration is equal to the target one, i.e. the extension leads to reach the target
		It is set to false in all other cases.
		* @param[out] return the reach target flag
		*/
		const bool&										Get_target_reached_flag() { return this->Get_extend_info()->target_reached; };
	protected:
		I_Tree() {};

		virtual	Node*									Extend(const Node* target) = 0;
		static  Node*									Extend_o(I_Tree* o, const Node* target) { return o->Extend(target); };

		virtual std::list<Node*>*						Get_Nodes() = 0;
		static  std::list<Node*>*						Get_Nodes_o(I_Tree* tree) { return tree->Get_Nodes(); };

		struct _extend_info {
			bool		random_or_deter;
			bool		target_reached;
		};
		virtual _extend_info*							Get_extend_info() = 0;
		static _extend_info*							Get_extend_info_o(I_Tree* o) { return o->Get_extend_info(); };
		bool											Extend_reached_determ_target();
	};



	/** \brief Interface for a concrete case of the decorator pattern modelling the Tree class.
	\details This interface actually contains a list of nodes, which can be made available for all the 
	wrapping decorators.
	*/
	class Tree_concrete : public I_Tree {
	public:
		/** \brief This constructor initializes the tree with an initial root node.
		* @param[in] root_state the state of the root node to consider for the tree.
		* @param[in] handler a reference to the object describing the planning problem to solve
		* @param[in] clone_handler when passed true, the handler is internally cloned and the cloned handler will be the one used by this class,
		which is in charge of destroying it when this object is destroyed. Otherwise the passed handler is used and will be not destroyed 
		when destroying this object.
		*/
		Tree_concrete(const Array& root_state, Node::I_Node_factory* handler, const bool& clone_handler);
		
		/** \brief Similar to Tree_concrete::Tree_concrete(const Node_State& root_state, Node::I_Node_factory* handler, const bool& clone_handler),
		but without inserting an initial root node.
		* @param[in] handler same as in Tree_concrete::Tree_concrete(const Node_State& root_state, Node::I_Node_factory* handler, const bool& clone_handler)
		* @param[in] clone_handler same as in Tree_concrete::Tree_concrete(const Node_State& root_state, Node::I_Node_factory* handler, const bool& clone_handler)
		*/
		Tree_concrete(Node::I_Node_factory* handler, const bool& clone_handler);

		~Tree_concrete();

		virtual Node::I_Node_factory*					Get_Problem_Handler() { return this->Problem_handler; };
	protected:
		virtual	Node*									Extend(const Node* target);

		virtual std::list<Node*>*						Get_Nodes() { return &this->Nodes; };

		virtual _extend_info*							Get_extend_info() { return &this->__ext_info; };

		virtual size_t									__get_Nodes_size() { return this->Nodes.size(); };
		virtual Node*									Nearest_Neighbour(const Node* state);
	// data
		bool												Problem_handler_was_cloned;
		Node::I_Node_factory*								Problem_handler;
		std::list<Node*>									Nodes;
		_extend_info										__ext_info;
	};



	/** \brief A decorator may contain another decorator or a concrete Tree
	*/
	class I_Tree_decorator : public I_Tree {
	public:
		/** \brief Constructor
		* @param[in] to_wrap the object to wrap inside this object
		* @param[in] destroy_wrap when passed true, to_wrap is deleted when deleting this object
		*/
		I_Tree_decorator(I_Tree* to_wrap, const bool& destroy_wrap) : Destroy_Wrapped(destroy_wrap), Wrapped(to_wrap) {};
		~I_Tree_decorator() { if (this->Destroy_Wrapped) delete this->Wrapped; };

		virtual Node::I_Node_factory*					Get_Problem_Handler() { return this->Wrapped->Get_Problem_Handler(); };
	protected:
		I_Tree* Get_Wrapped() { return this->Wrapped; };

		virtual	Node*									Extend(const Node* target) { return this->Extend_o(this->Wrapped, target); };
		virtual std::list<Node*>*						Get_Nodes() { return Get_Nodes_o(this->Wrapped); };
		virtual _extend_info*							Get_extend_info() { return this->Get_extend_info_o(this->Wrapped); };
	private:
	// data
		bool												Destroy_Wrapped;
		I_Tree*												Wrapped;
	};



	/** \brief This decorator perform the RRT* extension steps (near set computation, rewirds, etc. refer to Section 1.2.3 of the documentation).
	\details Such steps are automatically performed when invoking Extend_random or Extend_deterministic, when the extension was possible
	and a new node was actually inserted in the tree.
	*/
	class Tree_star : public I_Tree_decorator {
	public:
		/** \brief Constructor
		* @param[in] to_wrap the tree to decorate, for performing RRT* extensions
		* @param[in] destroy_wrap same meaning as in I_Tree_decorator::I_Tree_decorator
		*/
		Tree_star(I_Tree* to_wrap, const bool& destroy_wrap) : I_Tree_decorator(to_wrap, destroy_wrap) {};
		
		/** \brief This structure store the information regarding a possible rewird, i.e. a trajectory going
		from a starting node to an ending one, which are not already connected. cost is the cost to go
		from start to end.
		*/
		struct Node2Node_Traj {
			Node* start;
			Node* end;
			float cost;
		};

		/** \brief This method connect last_added node to its best father among the nodes in its near set and also
		evaluates the possible rewirds (Section 1.2.3 of the documentation) to do, wihtout performing it.
		\details  Basically, this function evaluates the rewirds, i.e. possible reconnection of the tree, for performing it at a second stage.
		* @param[out] possible_rewirds the possible rewirds to do for improving the connectivity
		* @param[out] last_added the node considered for the near set computation
		*/
		void											Connect_to_best_Father_and_eval_Rewirds(std::list<Node2Node_Traj>* possible_rewirds, Node* last_added);
	protected:
		virtual	Node*									Extend(const Node* target);

		virtual void									Near_set(std::list<Node*>* near_set, const Node* state);
	private:
#ifdef _REW_DEBUG
		void Print_Debug(Node* to_highlight);
#endif
	};

}
#endif