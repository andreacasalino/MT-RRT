
#ifndef __TREE_H__
#define __TREE_H__

#include "Node.h"

#ifdef _DEBUG
#define _DISPLAY_ITERATIONS //in debug mode, iteration counters are displayed
#else
#undef _DISPLAY_ITERATIONS
#endif // DEBUG

struct expansion_info {
	enum expansion_mode { single_tree, bi_directional, star };

	float					mDeterministic_coeff;
	int						mMax_iterations;
	bool					bEnable_termination; //set false to avoid terminating expansion (no matter of the deterministic coefficient)
	expansion_mode			mExp_mode;
	Node*					pStart_Node;
	Node*					pEnd_Node;
	list<VectorF>			mPath_solution;
};


class Tree_abstract
{
	friend class Tree_Decorator;
public:
	virtual ~Tree_abstract() {};
// methods
	virtual Node* Extend(Node* target, bool* trg_reached, const bool& is_a_possible_solution) = 0;
	virtual Node* Nearest_Neighbour(Node* target) = 0; //public only to let mpi_master use it when pock new roots
	virtual void  Interaction_with_other_nodes(const int& command, list<Node*>* newer_solutions) = 0; //used in parallel versions
	virtual Node* Get_last_node() = 0; //only for bidirectional mpi parallel_03

	Node* Extend_toward_random();
	struct Getter_factory {
		friend class I_Tree_Handler;
	private:
		static I_Node_Factory* Get_fctry(Tree_abstract* involved) { return involved->Get_Factory(); };
	};
// debug
	virtual void  Disp(ostream& file)= 0;
	virtual void  Disp_names(ostream& file) = 0; //only of log #slave produced nodes in parallel_04
protected:
// getters
	virtual I_Node_Factory* Get_Factory() = 0;
	virtual list<Node*>*	Get_Nodes() = 0;
};



class I_Tree_Handler
{
public:
	I_Tree_Handler(const unsigned int& ranks) : mElements(ranks) {};
	virtual void Init(const expansion_info& precomputed_info, I_Node_Factory* factory) = 0;
	~I_Tree_Handler() {
		delete this->pReference; 
		for (auto it = this->mOthers.begin(); it != this->mOthers.end(); it++) delete *it;
		delete this->mInfo.pStart_Node; delete this->mInfo.pEnd_Node;
	};
	// methods
	expansion_info::expansion_mode Get_mode() { return this->mInfo.mExp_mode; };
	void Disable_termination() { this->mInfo.bEnable_termination = false; };
	int expand();
	void Get_solution(list<VectorF>& percorso);

	void  Extract_path(Node* last_added, list<VectorF>& path) { this->Get_factory()->Extract_path(last_added, path); };
	void  Extract_path(Node* S, Node* E, list<VectorF>& path) { this->Get_factory()->Extract_path(S, E, path); };
	void extract_best_from_feasible(list<Node*> mFeasible_Solutions);

	virtual void Disp_Tree(string file_name);
	virtual void Disp_solution(string file_traj);
protected:
	// methods
	virtual Tree_abstract* Get_end_tree() { return this->mOthers.front(); };
	virtual int expand_single() = 0;
	virtual int expand_bidirectional() = 0;
	virtual int expand_star() = 0;
	I_Node_Factory* Get_factory() { return Tree_abstract::Getter_factory::Get_fctry(this->pReference); };
	// data
	unsigned int			mElements;
	Tree_abstract*			pReference;
	vector<Tree_abstract*>	mOthers;
	expansion_info			mInfo;
};


class Tree_Decorator : public Tree_abstract
{
public:
	// constructor
	Tree_Decorator(Tree_abstract* tree_wrapped) : pTree_inner(tree_wrapped), bInner_tree_to_destroy(true) {};
	~Tree_Decorator();
// methods
	Node* Extend(Node* target, bool* trg_reached, const bool& is_a_possible_solution) { return pTree_inner->Extend(target, trg_reached, is_a_possible_solution); };
	Node* Nearest_Neighbour(Node* target) { return pTree_inner->Nearest_Neighbour(target); };
	void  Interaction_with_other_nodes(const int& command, list<Node*>* newer_solutions) { pTree_inner->Interaction_with_other_nodes(command, newer_solutions); };
	Node* Get_last_node() { return pTree_inner->Get_last_node(); };
// debug
	void  Disp(ostream& file) { pTree_inner->Disp(file); };
	void  Disp_names(ostream& file) { pTree_inner->Disp_names(file); };
protected:
// getters
	I_Node_Factory* Get_Factory() { return pTree_inner->Get_Factory(); };
	list<Node*>*	Get_Nodes() { return pTree_inner->Get_Nodes(); };
// data
	Tree_abstract*  pTree_inner;
	bool			bInner_tree_to_destroy;
};



class Tree : public Tree_abstract
{
public:
// constructor
	Tree(Node* root, I_Node_Factory* factory, bool have_to_destroy_factory); // root must be always a copy 
	~Tree(); //always delete root node

	class Handler : public I_Tree_Handler {
	public:
		Handler() : I_Tree_Handler(1) {};
	protected:
		void Init(const expansion_info& precomputed_info, I_Node_Factory* factory);

		int expand_single();
		int expand_bidirectional();
		int expand_star();
	};
// methods
	Node*		  Extend(Node* target, bool* trg_reached, const bool& is_a_possible_solution);
	Node*		  Nearest_Neighbour(Node* target);
	void		  Interaction_with_other_nodes(const int& command, list<Node*>* newer_solutions) { abort(); };
	Node*		  Get_last_node() { return this->mNodes.back(); };
// debug
	void		  Disp(ostream& file);
	void		  Disp_names(ostream& file) { abort(); };
protected:
// getters
	I_Node_Factory* Get_Factory() { return this->pFactory; };
	list<Node*>*	Get_Nodes() { return  &this->mNodes; };
// data
	bool			bHave_to_destroy_factory;
	I_Node_Factory* pFactory;
	list<Node*>		mNodes;		//this list always refer to the complete list of nodes possessed by the tree
};



class star_search_decorator : public Tree_Decorator
{
public:
// constructor
	star_search_decorator(Tree_abstract* tree_wrapped);
	star_search_decorator(Tree_abstract* tree_wrapped, list<Node*>* containing_lists, I_Node_Factory* factory); //for building decorator_parallel_01
// methods
	Node* Extend(Node* target, bool* trg_reached, const bool& is_a_possible_solution);

	struct rewird_to_do_list {
		Node*			  involved;
		Node*			  new_fath;
		float			  new_cost;
	};
protected:
// methods
	void Rewird_lazy(Node* last_added); //rewird of existing nodes are computed but not applied: just put in mLast_rewired
	void Compute_and_Get_last_rewired(list<Node*>* container); //execute rewird cumulated in mLast_rewired and return the list of nodes involved
	void Rewird(Node* last_added, list<Node*>* container); //execute rewird and return nodes involved
	virtual void Near_Set(Node* last_added, list<Node*>* near_set);
// data
	I_Node_Factory*			pFactory; //computed in constructor once for all to avoid a recursive getter 
	list<list<Node*>*>		mPt_to_Nodes; //a set of distributed lists contains all the nodes
	float					mGamma;
	int						mState_dimension;
	list<rewird_to_do_list>	mLast_rewired;
};

#endif