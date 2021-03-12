/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Extender.h>
#include <Error.h>

#ifdef SHOW_PROGRESS
#include <iostream>
#include <omp.h>
#endif

namespace mt {
    std::vector<NodeState> convert(const std::list<const NodeState*> nodes) {
        std::vector<NodeState> result;
        result.reserve(nodes.size());
        for (auto it = nodes.begin(); it != nodes.end(); ++it) {
            result.emplace_back(**it);
        }
        return result;
    }

    TreeCore* convert(Tree* t) {
        TreeCore* temp = dynamic_cast<TreeCore*>(t);
        if(nullptr == temp) throw Error("conversion not possible");
        return temp;
    }

#ifdef SHOW_PROGRESS
    std::mutex ProgressPrinter::coutMtx;

    void ProgressPrinter::show(const std::size_t& iter) {
        if (0 == omp_get_thread_num()) {
            std::lock_guard<std::mutex> coutLock(coutMtx);
            std::cout << "iteration: " << iter << std::endl;
        }
    }
#endif
}