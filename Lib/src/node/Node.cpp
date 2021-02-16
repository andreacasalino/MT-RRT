/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <node/Node.h>
#include <Error.h>
using namespace std;

namespace mt::node {
	Node::Node(Node* father, const float& cost, const std::vector<float>& state)
		: state(state)
		, father(father)
		, costFromFather(cost) {
	}

	void Node::setFather(Node* new_father, const float& cost_from_father) {
		this->father = new_father;
		this->costFromFather = cost_from_father;
	}

	float Node::cost2Root(const size_t& I_max) const {
		float result = 0.f;
		size_t k = 0;
		const Node* att_node = this;
		while (att_node != nullptr) {
			++k;
			if (k == I_max) throw Error("Max number of iterations exceeded while computing cost to go");
			result += att_node->costFromFather;
			att_node = att_node->father;
		}
	}

	float Node::cost2Root() const {
		float result = 0.f;
		const Node* att_node = this;
		while (att_node != nullptr) {
			result += att_node->costFromFather;
			att_node = att_node->father;
		}
		return result;
	};
}