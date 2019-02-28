#ifndef __TREE_MT_02_H__
#define __TREE_MT_02_H__

#include "../Tree.h"
#include "Tree_Multi_Thread.h"
#include <omp.h>

////////////////////////////////////////////////////////////////
//parallelization of the exploring process with a single tree //
////////////////////////////////////////////////////////////////

class MultiT_shared_list : public Tree_Decorator
{
friend class MultiT_shared_list_star;
public:
	~MultiT_shared_list() { if (this->bInner_tree_to_destroy) delete this->pFactory; };

	class Handler : public I_MultiT_trees_Handler {
	public:
		Handler(const unsigned int& ranks) : I_MultiT_trees_Handler(ranks) {};
	protected:
		void Init(const expansion_info& precomputed_info, I_Node_Factory* factory);
		Tree_abstract* Get_end_tree() { return this->mOthers.back(); };
	};
// methods
	Node* Extend(Node* target, bool* trg_reached, const bool& is_a_possible_solution);
	Node* Nearest_Neighbour(Node* target);
	void  Interaction_with_other_nodes(const int& command, list<Node*>* newer_solutions) {};

protected:
	MultiT_shared_list(Tree* tree, const unsigned int& k_thread);
// data
	list<Node*>*						pNodes;
	I_Node_Factory*						pFactory;
};

class MultiT_shared_list_star : public star_search_decorator
{
public:
	MultiT_shared_list_star(MultiT_shared_list* tree);
// methods
	Node* Extend(Node* target, bool* trg_reached, const bool& is_a_possible_solution);
private:
// methods
	void Near_Set(Node* last_added, list<Node*>* near_set);
	void Rewird_with_critical_region(Node* last_added);
// data
	MultiT_shared_list*					pRef_to_inner_decorator;
};

#endif