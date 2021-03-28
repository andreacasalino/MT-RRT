/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/Pool.h"
#include <Error.h>

namespace mt::solver::qpar {
    Pool::Pool() {
        this->opened = std::make_shared<std::atomic_bool>(false);
    }

    Pool::~Pool() {
        this->close();
    }

    void Pool::JobExecutor::spin() {
        while (true == *this->opened) {
            std::lock_guard<std::mutex> jobLock(this->mtx);
            if (nullptr != this->job) {
                (*this->job)();
                this->job.reset();
            }
        }
    }

    void Pool::open(const std::size_t& size) {
        if (true == *this->opened) {
            throw Error("Thread pool already opened");
        }
        *this->opened = true;

        this->jobs.reserve(size);
        this->threads.reserve(size);
        for (std::size_t k = 0; k < size; ++k) {
            this->jobs.emplace_back(this->opened);
            this->threads.emplace_back(&JobExecutor::spin, &this->jobs.back());
        }
    }

    void Pool::close() {
        if (false == *this->opened) {
            return;
        }
        *this->opened = false;

        for (std::size_t k = 0; k < this->jobs.size(); ++k) {
            this->threads[k].join();
        }
        this->threads.clear();
        this->jobs.clear();
    }

    void Pool::addJob(const std::vector<Job>& jobs) {
        for (std::size_t k = 0; k < jobs.size(); ++k) {
            std::lock_guard<std::mutex> jobLock(this->jobs[k].mtx);
            this->jobs[k].job = std::make_unique<Job>(jobs[k]);
        }
    }

    void Pool::wait() {
        if (false == *this->opened) {
            return;
        }
        for (auto it = this->jobs.begin(); it != this->jobs.end(); ++it) {
            while (true) {
                std::lock_guard<std::mutex> jobLock(it->mtx);
                if (nullptr == it->job) {
                    break;
                }
            }
        }
    }
}