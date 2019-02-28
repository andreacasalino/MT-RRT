#ifndef __TREE_MT_04_H__
#define __TREE_MT_04_H__

#include "../Tree.h"
#include <omp.h>

//////////////////////////
//multi agents approach //
//////////////////////////

class MultiT_slave_decorator;
class MultiT_master_decorator : public Tree_Decorator {
public:
	class Handler : public I_Tree_Handler {
	public:
		Handler(const unsigned int& ranks, const float bat_prtcg) : I_Tree_Handler(ranks), batch_prctg(bat_prtcg) {};
	protected:
		void Init(const expansion_info& precomputed_info, I_Node_Factory* factory);

		int expand_single();
		int expand_bidirectional() { abort(); };
		int expand_star();
	private:
		void blind_extensions(Tree_abstract* T_ref, Node* N_end_copia, const float& alfa);
	// data
		float batch_prctg;
		int   batch_size;
	};

	MultiT_master_decorator(Node* root, I_Node_Factory* factory, const int& N_threads, list<MultiT_slave_decorator*>* slave_created);
protected:
// methods
	void  Interaction_with_other_nodes(const int& command, list<Node*>* newer_solutions);

	void Sample_new_roots();
	void Compute_T_for_rewirds();
	void Process_nodes(list<Node*>* newer_solutions);
	void Process_rewirds();
// data
	list<Node*>*					pWrapped_nodes;
	list<float>						mW_4_sampling;
	float							mW_sum;
	list<MultiT_slave_decorator*>	p_slaves;
};

class MultiT_slave_decorator : public Tree_Decorator {
	friend class MultiT_master_decorator;
	friend class MultiT_slave_decorator_star;
public:
	// methods
	Node* Extend(Node* target, bool* trg_reached, const bool& is_a_possible_solution); //just to assign the name
	void  Interaction_with_other_nodes(const int& command, list<Node*>* newer_solutions);
private:
	MultiT_slave_decorator(Tree* to_wrap, const int& th_id);
// data
	int													mth_ID;
	Node*												pN_end;
	list<Node*>											mTo_consider_for_rewirds;
	list<star_search_decorator::rewird_to_do_list>		mRewird_cumulated;
	list<float>											mW_4_sampling;
};

class MultiT_slave_decorator_star : public star_search_decorator {
public:
	MultiT_slave_decorator_star(MultiT_slave_decorator* inner_slave);
// methods
	Node* Extend(Node* target, bool* trg_reached, const bool& is_a_possible_solution); //execute extend basic + lazy rewird
	void  Interaction_with_other_nodes(const int& command, list<Node*>* newer_solutions); //only for exploiting near set computation
private:
	MultiT_slave_decorator* pInner_slave;
};

#endif