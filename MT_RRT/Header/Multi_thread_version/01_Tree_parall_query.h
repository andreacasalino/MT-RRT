#ifndef __TREE_MT_01_H__
#define __TREE_MT_01_H__

#include "../Tree.h"
#include <omp.h>

////////////////////////////////////////////////////////////
//parallelization of nearest neighbout and near set search//
////////////////////////////////////////////////////////////

class MultiT_parallelized_search : public Tree_Decorator
{
	friend class MultiT_parallelized_search_star;
public:
	class Handler : public I_Tree_Handler {
		friend class MultiT_parallelized_search;
		friend class MultiT_parallelized_search_star;
	public:
		Handler(const unsigned int& ranks);
		~Handler();
		int expand_single();
		int expand_bidirectional();
		int expand_star();
	private:
		void Init(const expansion_info& precomputed_info, I_Node_Factory* factory);

		Node* Parallel_Nearest_Neighbour(const int& th_id); 
		void  Parallel_Near_Set(list<Node*>* near_set, const int& th_id);

		void thread_slave_duty(int th_id);

	// data
		int							mAttual_command;
		list<Node*>*				pt_Attual_list;
		Node*						pAttual;
		float						mAttual_Gamma;

		struct info_thread {
			I_Node_Factory*			pfactory;
			Node*					nearest;
			float					min_dist;
			list<Node*>				near_sets;
		};
		vector<info_thread>			Thread_results;
	};
	Node* Extend(Node* target, bool* trg_reached, const bool& is_a_possible_solution); //to use a different Nearest_Neighbour
private:
	MultiT_parallelized_search(Tree_abstract* inner_tree) : Tree_Decorator(inner_tree) {};
// methods
	Node* Nearest_Neighbour(Node* target);
// data
	MultiT_parallelized_search::Handler* pt_to_handler;
};

class MultiT_parallelized_search_star : public star_search_decorator
{
public:
	MultiT_parallelized_search_star(MultiT_parallelized_search* inner_tree) : star_search_decorator(inner_tree) , pInner_tree(inner_tree) {};
private:
	void Near_Set(Node* last_added, list<Node*>* near_set);
// data
	MultiT_parallelized_search*   pInner_tree;
};

#endif