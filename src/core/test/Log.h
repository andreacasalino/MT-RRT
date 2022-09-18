#pragma once

#ifdef TEST_LOGGING
#include <MT-RRT-core/../../src/Extender.h>
#include <TrivialProblemJson.h>

void log_test_case(mt_rrt::Extender& subject, const std::string& tag);
#endif
