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
		Pool();
		~Pool();

		void open(const std::size_t& size);
		void close();

		void addJob(const std::vector<Job>& jobs);
		void wait();

	private:
		typedef std::unique_ptr<Job> JobPtr;

		class JobExecutor {
		public:
			JobExecutor(JobExecutor&& o) noexcept : opened(o.opened) {}; //required to place it in a vector
			JobExecutor(std::shared_ptr<std::atomic_bool> opened) noexcept : opened(opened) {};

			void spin();

			std::mutex mtx;
			JobPtr job;

		private:
			std::shared_ptr<std::atomic_bool> opened;
		};

		std::shared_ptr<std::atomic_bool> opened;
		std::vector<JobExecutor> jobs;
		std::vector<std::thread> threads;
	};
}

#endif