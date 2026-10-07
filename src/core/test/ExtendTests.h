/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <gtest/gtest.h>

#include <MT-RRT/Extender.h>
#include <MT-RRT/ExtenderBidir.h>
#include <MT-RRT/ExtenderSingle.h>

#include <LogResult.h>
#include <Logger.h>

#include <optional>

namespace mt_rrt::testing {
template <typename TheExtender> struct ExtendTest : ::testing::Test {
  void TearDown() override {
    LogResult recipient;
    if constexpr (requires { extender->tree_; }) {
      nlohmann::to_json(recipient.addToScene("start"), extender->target());
      nlohmann::to_json(recipient.addToScene("end"),
                        extender->tree_.root()->data().state);
      recipient.addTree(extender->tree_.getNodes());
    } else {
      nlohmann::to_json(recipient.addToScene("start"),
                        extender->trees_.first.root()->data().state);
      nlohmann::to_json(recipient.addToScene("end"),
                        extender->trees_.second.root()->data().state);
      recipient.addTree(extender->trees_.first.getNodes());
      recipient.addTree(extender->trees_.second.getNodes());
    }
    for (auto &&solution : extender->materializeAllSolutions()) {
      recipient.addSolution(solution);
    }

    to_json(recipient, extender->getProblem().first.connector->getChecker());
    Logger::get().add_test_result(recipient.get());
  }

  std::optional<TheExtender> extender;
};

} // namespace mt_rrt::testing
