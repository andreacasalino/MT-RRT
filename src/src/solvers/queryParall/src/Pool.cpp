/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../Pool.h"
#include <Error.h>

namespace mt::qpar {
    void Pool::JobExecutor::spin() {
        while (opened == this->state) {
            std::lock_guard<std::mutex> jobLock(this->mtx);
            if (nullptr != this->job) {
                (*this->job)();
                this->job.reset();
            }
        }

    }

    void Pool::open(const std::size_t& size) {
        if (opened == this->state) {
            throw Error("Thread pool already opened");
        }
        this->state = opened;

        this->jobs.reserve(size);
        this->threads.reserve(size);
        for (std::size_t k = 0; k < size; ++k) {
            this->jobs.emplace_back(this->state);
            this->threads.emplace_back(&JobExecutor::spin, &this->jobs.back());
        }
    }

    void Pool::close() {
        if (closed == this->state) {
            return;
        }
        this->state = closed;

        for (std::size_t k = 0; k < this->jobs.size(); ++k) {
            this->threads[k].join();
        }
        this->jobs.clear();
        this->threads.clear();
    }

    Pool::~Pool() {
        this->close();
    }

    void Pool::addJob(const std::vector<Job>& jobs) {
        if (closed == this->state) {
            throw Error("Thread pool not opened before dispatching jobs");
        }
        for (std::size_t k = 0; k < jobs.size(); ++k) {
            std::lock_guard<std::mutex> jobLock(this->jobs[k].mtx);
            this->jobs[k].job = std::make_unique<Job>(jobs[k]);
        }
    }

    void Pool::wait() {
        if (closed == this->state) {
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