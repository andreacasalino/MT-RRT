/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_POOL_H
#define MT_RRT_POOL_H

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
		void addJob(const std::vector<Job>& jobs);
		void wait();

	private:
		typedef std::unique_ptr<Job> JobPtr;
		struct JobInfo {
			std::mutex mtx;
			JobPtr job;
		};
		typedef std::unique_ptr<JobInfo> JobInfoPtr;

		std::atomic_bool life = false;
		std::vector<JobInfoPtr> jobs;
		std::vector<std::thread> threads;
	};
}

#endif