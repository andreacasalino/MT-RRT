#ifndef __TREE_MT_H__
#define __TREE_MT_H__

#include "../Tree.h"
#include <omp.h>

class I_MultiT_trees_Handler : public I_Tree_Handler {
protected:
	I_MultiT_trees_Handler(const unsigned int& ranks) : I_Tree_Handler(ranks) {};
	int expand_single();
	int expand_bidirectional();
	int expand_star();
};

#endif