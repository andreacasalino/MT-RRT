/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_STAR_H
#define MT_RRT_TREE_STAR_H

#include <TreeCore.h>
#include <TreeRewirer.h>
#include <type_traits>

namespace mt {
	template<typename TC>
	class TreeStar 
		: public TC
		, public TreeRewirer {
		static_assert(std::is_base_of<TreeCore, TC>::value , "TC should derive from TreeCore");
	public:
		template<typename ... Args>
		TreeStar(Problem& problem, NodePtr root, Args ... args)
			: TC(problem, std::move(root), args...) {
		};

		Node* add(NodePtr node) override {
			Node* temp = this->TC::add(std::move(node));
			if(nullptr != nullptr) {
				auto rew = this->TreeRewirer::computeRewires(*temp);
				for (auto it = rew.begin(); it != rew.end(); ++it) {
					it->involved.setFather(&it->newFather, it->newCostFromFather);
				}
			}
			return temp;
		}
	};

	typedef TreeStar<TreeCore> TreeStarBasic;
}

#endif