/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../Tree.h"

namespace mt::qpar {
    void Pool::open(const std::size_t& size) {
        if (this->life) return;
        this->life = true;
        std::size_t thId = 0;
        auto loop = [this, thId]() {
            while (this->life) {
                std::lock_guard<std::mutex> jobLock(this->jobs[thId].mtx);
                if (nullptr != this->jobs[thId].job) {
                    (*this->jobs[thId].job.get())();
                    this->jobs[thId].job.reset();
                }
            }
        };

        for (std::size_t k = 0; k < size; ++k) {
            this->jobs.emplace_back();
            this->threads.emplace_back(loop);
            ++thId;
        }
    }

    void Pool::close() {
        if (!this->life) return;
        this->life = false;
        for (std::size_t k = 0; k < this->jobs.size(); ++k) {
            this->threads[k].join();
        }
    }

    Pool::~Pool() {
        if (this->life) {
            this->close();
        }
    }

    void Pool::addJob(const Job& job, const std::size_t& thId) {
        std::lock_guard<std::mutex> jobLock(this->jobs[thId].mtx);
        this->jobs[thId].job = std::make_unique<Job>(job);
    }

    void Pool::wait() {
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