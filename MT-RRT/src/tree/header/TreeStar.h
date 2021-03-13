/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_STAR_H
#define MT_RRT_TREE_STAR_H

#include <TreeBase.h>
#include <TreeRewirer.h>
#include <type_traits>

namespace mt {
	template<typename TBase>
	class TreeStar 
		: public TBase
		, public TreeRewirer {
		static_assert(std::is_base_of<TreeBase, TBase>::value , "TC should derive from TreeCore");
	public:
		template<typename ... Args>
		TreeStar(Args&&... args)
			: TBase(std::forward<Args>(args)...) {
		};

		Node* add(NodePtr node) override {
			if(nullptr != node) {
				auto rew = this->TreeRewirer::computeRewires(*node);
				for (auto it = rew.begin(); it != rew.end(); ++it) {
					it->involved.setFather(&it->newFather, it->newCostFromFather);
				}
			}
			return this->TBase::add(std::move(node));
		}
	};
}

#endif