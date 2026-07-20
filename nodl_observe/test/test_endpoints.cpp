// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Layer-1 unit tests for the endpoint builders.  Ports
// test_observe.py::TestTopicEndpoints and ::TestServiceEndpoints.
//
// NOTE: rclcpp::TopicEndpointInfo has no practical public constructor for a
// hand-rolled fixture, so the QoS+type-hash-carrying path (the Python
// FakeEndpointInfo case) is exercised through the raw-fields build_topic
// overload rather than build_topic_endpoints.  build_topic_endpoints is still
// covered for its info-less (name/type-only) and sorting behaviour with an
// empty infos map -- which is the only branch reachable without a live graph.

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

#include "nodl_observe/endpoints.hpp"
#include "rclcpp/qos.hpp"
#include "rosgraph_msgs/msg/qo_s_profile.hpp"
#ifndef ROS2_HUMBLE
  #include "rosidl_runtime_c/type_hash.h"  // REP-2011 type hashes (Iron+ only)
#endif

using nodl_observe::build_service_endpoints;
using nodl_observe::build_topic;
using nodl_observe::build_topic_endpoints;
using QoSMsg = rosgraph_msgs::msg::QoSProfile;

#ifndef ROS2_HUMBLE
namespace
{

rosidl_type_hash_t make_hash(uint8_t byte_value)
{
  rosidl_type_hash_t h{};
  h.version = 1;
  for (size_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; ++i) {
    h.value[i] = byte_value;
  }
  return h;
}

}  // namespace

// Iron+ only: pre-Iron has no REP-2011 type hash, so the hash-carrying path does
// not exist there (build_topic has no hash overload).  Humble leaves the hash at
// the message default, which the service-endpoint test below already covers.
TEST(TopicEndpoints, TopicCarriesTypeHashAndQos)
{
  const auto hash = make_hash(0xAB);
  const auto t = build_topic("/chatter", "std_msgs/msg/String", rclcpp::QoS(10).best_effort(), &hash);

  EXPECT_EQ(t.name, "/chatter");
  EXPECT_EQ(t.type.name, "std_msgs/msg/String");
  EXPECT_EQ(t.type.hash.version, 1);
  for (size_t i = 0; i < t.type.hash.value.size(); ++i) {
    EXPECT_EQ(t.type.hash.value[i], 0xAB);
  }
  EXPECT_EQ(t.qos.reliability, QoSMsg::RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(t.qos.depth, 10u);
}
#endif  // ROS2_HUMBLE

TEST(TopicEndpoints, TopicWithoutInfoIsNameTypeOnly)
{
  std::map<std::string, std::vector<std::string>> nat{{"/chatter", {"std_msgs/msg/String"}}};
  const auto topics = build_topic_endpoints(nat, {});
  ASSERT_EQ(topics.size(), 1u);
  EXPECT_EQ(topics[0].name, "/chatter");
  EXPECT_EQ(topics[0].type.name, "std_msgs/msg/String");
  EXPECT_EQ(topics[0].qos.depth, 0u);
}

TEST(TopicEndpoints, TopicsSortedByNameThenType)
{
  std::map<std::string, std::vector<std::string>> nat{{"/b", {"t/B"}}, {"/a", {"t/A"}}};
  const auto topics = build_topic_endpoints(nat, {});
  ASSERT_EQ(topics.size(), 2u);
  EXPECT_EQ(topics[0].name, "/a");
  EXPECT_EQ(topics[1].name, "/b");
}

TEST(ServiceEndpoints, ServiceHasUnknownQosAndNoHash)
{
  std::map<std::string, std::vector<std::string>> nat{{"/add", {"example_interfaces/srv/AddTwoInts"}}};
  const auto services = build_service_endpoints(nat);
  ASSERT_EQ(services.size(), 1u);
  const auto & s = services[0];
  EXPECT_EQ(s.name, "/add");
  EXPECT_EQ(s.request_type.name, "example_interfaces/srv/AddTwoInts");
  EXPECT_EQ(s.response_type.name, "example_interfaces/srv/AddTwoInts");
  // Type hash unset -> message default (version 1 per TypeHash.msg) with an
  // all-zero value.
  for (size_t i = 0; i < s.request_type.hash.value.size(); ++i) {
    EXPECT_EQ(s.request_type.hash.value[i], 0u);
  }
  EXPECT_EQ(s.request_qos.reliability, QoSMsg::RELIABILITY_UNKNOWN);
  EXPECT_EQ(s.response_qos.durability, QoSMsg::DURABILITY_UNKNOWN);
}

TEST(ServiceEndpoints, ServicesSorted)
{
  std::map<std::string, std::vector<std::string>> nat{{"/z", {"t/Z"}}, {"/a", {"t/A"}}};
  const auto services = build_service_endpoints(nat);
  ASSERT_EQ(services.size(), 2u);
  EXPECT_EQ(services[0].name, "/a");
  EXPECT_EQ(services[1].name, "/z");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
