// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Degradation test for collect_parameters (the one part of observation that
// contacts the target node).  Ports test_observe.py::TestParameterDegradation.
// Unlike the pure pairing logic in test_parameters.cpp, this needs a live rclcpp
// context + executor, so it lives in its own gtest: the graceful-degradation
// contract is that an unresponsive / absent target yields empty arrays and a
// logged warning, never an exception.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "nodl_observe/parameters.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

TEST(CollectParameters, AbsentTargetDegradesToEmpty)
{
  auto node = std::make_shared<rclcpp::Node>("_nodl_observe_param_degradation_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  // No node provides parameter services for this FQN, so the client never
  // becomes ready within the short ceiling -> the graceful-degradation path
  // returns empty arrays (with a logged warning) rather than throwing.
  const auto [descriptors, values] =
    nodl_observe::collect_parameters(*node, executor, "/nodl_observe_absent_target", 0.3s);

  EXPECT_TRUE(descriptors.empty());
  EXPECT_TRUE(values.empty());

  executor.remove_node(node->get_node_base_interface());
}

TEST(CollectParameters, ZeroTimeoutDegradesToEmpty)
{
  auto node = std::make_shared<rclcpp::Node>("_nodl_observe_param_zero_timeout_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  // A non-positive remaining budget must short-circuit to empty, never block.
  const auto [descriptors, values] = nodl_observe::collect_parameters(*node, executor, "/any_target", 0.0s);

  EXPECT_TRUE(descriptors.empty());
  EXPECT_TRUE(values.empty());

  executor.remove_node(node->get_node_base_interface());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
