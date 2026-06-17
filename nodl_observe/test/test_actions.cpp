// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Layer-1 unit tests for action folding.  Ports
// test_observe.py::TestActionFolding.  Pure: feeds the builders names-and-types
// maps; no graph, no rcl_action.

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

#include "nodl_observe/endpoints.hpp"
#include "rosgraph_msgs/msg/qo_s_profile.hpp"

using nodl_observe::build_service_endpoints;
using nodl_observe::build_topic_endpoints;
using nodl_observe::fold_actions;
using QoSMsg = rosgraph_msgs::msg::QoSProfile;

namespace
{

using NamesAndTypes = std::map<std::string, std::vector<std::string>>;

// Build the full set of /fib constituents (3 services + 2 topics) plus one
// unrelated service and topic, as in the Python _full_constituents helper.
void full_constituents(
  const std::string & action,
  std::vector<rosgraph_msgs::msg::Service> & services,
  std::vector<rosgraph_msgs::msg::Topic> & topics)
{
  NamesAndTypes srv_nat{
    {action + "/_action/send_goal", {"t/SendGoal"}},
    {action + "/_action/get_result", {"t/GetResult"}},
    {action + "/_action/cancel_goal", {"t/CancelGoal"}},
    {"/other_service", {"t/Other"}},
  };
  services = build_service_endpoints(srv_nat);

  NamesAndTypes topic_nat{
    {action + "/_action/feedback", {"t/Feedback"}},
    {action + "/_action/status", {"t/Status"}},
    {"/other_topic", {"t/OtherTopic"}},
  };
  topics = build_topic_endpoints(topic_nat, {});
}

}  // namespace

TEST(ActionFolding, ConstituentsFoldedAndRemovedFromFlat)
{
  std::vector<rosgraph_msgs::msg::Service> services;
  std::vector<rosgraph_msgs::msg::Topic> topics;
  full_constituents("/fib", services, topics);

  NamesAndTypes actions_nat{{"/fib", {"action_tutorials_interfaces/action/Fibonacci"}}};
  const auto actions = fold_actions(actions_nat, services, topics);

  ASSERT_EQ(actions.size(), 1u);
  const auto & a = actions[0];
  EXPECT_EQ(a.name, "/fib");
  EXPECT_EQ(a.send_goal.name, "/fib/_action/send_goal");
  EXPECT_EQ(a.get_result.name, "/fib/_action/get_result");
  EXPECT_EQ(a.cancel_goal.name, "/fib/_action/cancel_goal");
  EXPECT_EQ(a.feedback.name, "/fib/_action/feedback");
  EXPECT_EQ(a.status.name, "/fib/_action/status");

  // Folded constituents must NOT remain flat.
  ASSERT_EQ(services.size(), 1u);
  EXPECT_EQ(services[0].name, "/other_service");
  ASSERT_EQ(topics.size(), 1u);
  EXPECT_EQ(topics[0].name, "/other_topic");

  // Folded services keep UNKNOWN QoS.
  EXPECT_EQ(a.send_goal.request_qos.reliability, QoSMsg::RELIABILITY_UNKNOWN);
}

TEST(ActionFolding, OrphanActionEntityStaysFlat)
{
  // An _action/* service whose parent action is NOT in the action graph must be
  // left flat, never silently discarded.
  NamesAndTypes srv_nat{{"/ghost/_action/send_goal", {"t/SendGoal"}}};
  auto services = build_service_endpoints(srv_nat);
  std::vector<rosgraph_msgs::msg::Topic> topics;

  const auto actions = fold_actions({}, services, topics);
  EXPECT_TRUE(actions.empty());
  ASSERT_EQ(services.size(), 1u);
  EXPECT_EQ(services[0].name, "/ghost/_action/send_goal");
}

TEST(ActionFolding, PartialActionUsesPlaceholders)
{
  NamesAndTypes srv_nat{{"/fib/_action/send_goal", {"t/SendGoal"}}};
  auto services = build_service_endpoints(srv_nat);
  NamesAndTypes topic_nat{{"/fib/_action/feedback", {"t/Feedback"}}};
  auto topics = build_topic_endpoints(topic_nat, {});

  NamesAndTypes actions_nat{{"/fib", {"t/Action"}}};
  const auto actions = fold_actions(actions_nat, services, topics);

  ASSERT_EQ(actions.size(), 1u);
  const auto & a = actions[0];
  EXPECT_EQ(a.send_goal.name, "/fib/_action/send_goal");
  // Missing constituents get placeholder names.
  EXPECT_EQ(a.get_result.name, "/fib/_action/get_result");
  EXPECT_EQ(a.status.name, "/fib/_action/status");
  EXPECT_TRUE(services.empty());
  EXPECT_TRUE(topics.empty());
}

TEST(ActionFolding, ActionsSorted)
{
  std::vector<rosgraph_msgs::msg::Service> services;
  std::vector<rosgraph_msgs::msg::Topic> topics;
  NamesAndTypes actions_nat{{"/z", {"t/Z"}}, {"/a", {"t/A"}}};
  const auto actions = fold_actions(actions_nat, services, topics);
  ASSERT_EQ(actions.size(), 2u);
  EXPECT_EQ(actions[0].name, "/a");
  EXPECT_EQ(actions[1].name, "/z");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
