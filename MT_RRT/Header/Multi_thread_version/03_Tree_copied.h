#ifndef __TREE_MT_03_H__
#define __TREE_MT_03_H__

#include "../Tree.h"
#include "Tree_Multi_Thread.h"
#include <omp.h>


///////////////////////////////////////////////////////////////////////
//parallelization of the exploring process with different list nodes //
///////////////////////////////////////////////////////////////////////

template <typename ElementType>
class Linked_received_info { //for received nodes or rewirds
public:
	static list<Linked_received_info<ElementType>*> Create_linked_group(const int& N_threads) {
		if (N_threads < 2) abort();
		list<Linked_received_info<ElementType>*> group;
		for (int k = 0; k < N_threads; k++)
			group.push_back(new Linked_received_info<ElementType>(N_threads));

		int k1 = 0, k2, k_buff;
		int k;
		//typename  list<Linked_received_info<ElementType>*>::iterator it_group2;
		for (auto it_group = group.begin(); it_group != group.end(); it_group++) {
			(*it_group)->pOutgoing_buffer.reserve(N_threads - 1);
			k_buff = 0;
			for (auto it_group2 = group.begin(); it_group2 != group.end(); it_group2++) {
				if (*it_group2 != *it_group) {
					k = 0;
					for (k2 = 0; k2 < N_threads; k2++) {
						if (k2 == k1) {
							(*it_group)->pOutgoing_buffer.push_back(&(*it_group2)->Incoming_buffer[k]);
						}
						if (k2 != k_buff)
							k++;
					}
				}
				k_buff++;
			}
			k1++;
		}

		return group;
	};

	int size() { return (int)this->Incoming_buffer.size(); };
	void Gather_info(list<ElementType>* gathered, bool* something_remain) { //return false if all Incomings reached end position
		gathered->clear();
		*something_remain = false;
		auto it_cursor = this->Incoming_buffer_cursor.begin();
		auto it_next = *it_cursor;
		for (auto it_In = this->Incoming_buffer.begin(); it_In != this->Incoming_buffer.end(); it_In++) {
			it_next = *it_cursor; it_next++;
			if (it_next != it_In->end()) {
				gathered->push_back(*it_next);
				*it_cursor = it_next;
				*something_remain = true;
			}
			it_cursor++;
		}
	};
	void Dispatch_info(list<ElementType>& sending_list) {
		auto it_send = sending_list.begin();
		for (auto it_Out = this->pOutgoing_buffer.begin(); it_Out != this->pOutgoing_buffer.end(); it_Out++) {
			(*it_Out)->push_back(*it_send);
			it_send++;
		}
	};
private:
	// constructor
	Linked_received_info<ElementType>(const int& N_threads) {
		this->Incoming_buffer.reserve(N_threads - 1);
		this->Incoming_buffer_cursor.reserve(N_threads - 1);
		int k;
		for (k = 0; k < (N_threads - 1); k++) {
			this->Incoming_buffer.push_back(list<ElementType>());
			this->Incoming_buffer.back().push_back(NULL);
			this->Incoming_buffer_cursor.push_back(this->Incoming_buffer.back().begin());
		}
	};
	// data
	vector<list<ElementType>>						Incoming_buffer;
	vector<typename list<ElementType>::iterator>    Incoming_buffer_cursor;
	vector<list<ElementType>*>						pOutgoing_buffer;
};

class MultiT_copied_list_decorator : public Tree_Decorator
{
friend class MultiT_copied_list_decorator_star;
public:
	~MultiT_copied_list_decorator() { delete this->pNodes_storage; };

	class Handler : public I_MultiT_trees_Handler {
	public:
		Handler(const unsigned int& ranks) : I_MultiT_trees_Handler(ranks) {};
	protected:
		void Init(const expansion_info& precomputed_info, I_Node_Factory* factory);
		Tree_abstract* Get_end_tree() { return this->mOthers.back(); };
	};
// methods
	Node* Extend(Node* target, bool* trg_reached, const bool& is_a_possible_solution);
	void  Interaction_with_other_nodes(const int& command, list<Node*>* newer_solutions);
protected:
	static vector<MultiT_copied_list_decorator*> Create_linked_group(vector<Tree*>& group);
	MultiT_copied_list_decorator(Tree* tree_inner, Linked_received_info<Node*>* recv_nodes);
// methods
	void  Process_received_nodes_to_add(bool* something_remain); 
// data
	Linked_received_info<Node*>*		pNodes_storage;
	list<Node*>*						pNodes;
};

class MultiT_copied_list_decorator_star : public star_search_decorator
{
public:
	~MultiT_copied_list_decorator_star() { delete this->pRewird_storage; };
	static vector<MultiT_copied_list_decorator_star*> Create_linked_group(vector<MultiT_copied_list_decorator*>& group);
// methods
	Node* Extend(Node* target, bool* trg_reached, const bool& is_a_possible_solution);
	void  Interaction_with_other_nodes(const int& command, list<Node*>* newer_solutions);
private:
	struct rew_query_el {
		Node*			  Node_to_modify;
		Node*			  New_father_to_set;
		float			  New_cost_from_father;
	};

	MultiT_copied_list_decorator_star(MultiT_copied_list_decorator* tree_inner, Linked_received_info<rew_query_el*>* recv_rew, int& my_Th_iD);
// methods
	void  Process_received_rewird_to_do(bool* something_remain); 
// data

	MultiT_copied_list_decorator*			pRef_to_inner_omp_decorator;
	Linked_received_info<rew_query_el*>*	pRewird_storage;
	int										mThread_ID;
};

#endif