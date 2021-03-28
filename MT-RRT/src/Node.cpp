/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Node.h>
#include <Error.h>
#include <limits>
using namespace std;

namespace mt {
	constexpr std::size_t MAX_ITERATIONS = std::numeric_limits<std::size_t>::max();

	Node::Node(const NodeState& state)
		: state(state) {
		if (this->state.empty()) throw Error("empty state not valid for describing node state");
	}

	void Node::setFather(Node* new_father, const float& cost_from_father) {
		this->father = new_father;
		this->costFromFather = cost_from_father;
	}

	float Node::cost2Root() const {
		float result = 0.f;
		const Node* att_node = this;
		size_t k = 0;
		while (att_node != nullptr) {
			result += att_node->costFromFather;
			att_node = att_node->father;
			if(MAX_ITERATIONS == ++k) throw Error("Max number of iterations exceeded while computing cost to go");
		}
		return result;
	};
}