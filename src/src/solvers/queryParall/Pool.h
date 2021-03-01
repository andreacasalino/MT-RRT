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
	typedef std::function<void(void)> Job;

	class Pool {
	public:
		Pool() = default;
		~Pool();

		void open(const std::size_t& size);
		void close();

		void addJob(const std::vector<Job>& jobs);
		void wait();

	private:
		enum State { closed, opened };

		typedef std::unique_ptr<Job> JobPtr;

		class JobExecutor {
		public:
			JobExecutor(JobExecutor&& o) noexcept : state(o.state) {}; //required to place it in a vector
			JobExecutor(std::atomic<State>& state) noexcept : state(state) {};

			void spin();

			std::mutex mtx;
			JobPtr job;

		private:
			std::atomic<State>& state;
		};

		std::atomic<State> state = closed;
		std::vector<JobExecutor> jobs;
		std::vector<std::thread> threads;
	};
}

#endif