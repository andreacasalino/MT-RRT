/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_QPAR_TREE_H
#define MT_RRT_QPAR_TREE_H

#include <TreeConcrete.h>
#include <thread>
#include <functional>
#include <mutex>
#include <atomic>

namespace mt::qpar {
	class Pool {
	public:
		Pool() = default;
	   ~Pool();

		void open(const std::size_t& size);
		void close();

		typedef std::function<void(void)> Job;
		void addJob(const Job& job, const std::size_t& thId);
		void wait();

	private:
		typedef std::unique_ptr<Job> JobPtr;
		struct JobInfo {
			std::mutex mtx;
			JobPtr job;
		};

		std::atomic_bool life = false;
		std::vector<JobInfo> jobs;
		std::vector<std::thread> threads;
	};

	class Tree : public TreeConcrete {
	public:
		Tree(const std::vector<ProblemPtr>& problems, NodePtr root);
		Tree(const Tree& o, NodePtr root);

		inline void open() { this->pool->open(this->problems.size()); };
		inline void close() { this->pool->close(); };

	private:
		Node* nearestNeighbour(const NodeState& state) const override;

		std::set<Node*> nearSet(Node& node) const override;

		std::vector<Problem*> problems;
		std::shared_ptr<Pool> pool;
	};
}

#endif