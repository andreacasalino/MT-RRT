#ifdef TEST_LOGGING

#include "Log.h"

void log_test_case(mt_rrt::Extender& subject, const std::string& tag) {
    mt_rrt::utils::Logger::Log log;
    log.tag = tag;
    mt_rrt::samples::TrivialProblemConverter::CONVERTER.toJson2(log.content, subject);
    log.python_visualizer = mt_rrt::utils::default_python_sources(TRIVIAL_PROBLEM_PYTHON_SCRIPT);
    mt_rrt::utils::Logger::log(log);
}

#endif
