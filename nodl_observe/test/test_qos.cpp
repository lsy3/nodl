// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Layer-1 unit tests for the QoS mapping (no executor, no graph).  Ports
// test_observe.py::TestQoSMapping.

#include <gtest/gtest.h>

#include "nodl_observe/qos.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"
#include "rosgraph_msgs/msg/qo_s_profile.hpp"

using nodl_observe::qos_to_msg;
using nodl_observe::unknown_qos_msg;
using QoSMsg = rosgraph_msgs::msg::QoSProfile;

TEST(QoSMapping, History)
{
  EXPECT_EQ(
    qos_to_msg(rclcpp::QoS(1).history(rclcpp::HistoryPolicy::SystemDefault)).history, QoSMsg::HISTORY_SYSTEM_DEFAULT);
  EXPECT_EQ(qos_to_msg(rclcpp::QoS(rclcpp::KeepLast(1))).history, QoSMsg::HISTORY_KEEP_LAST);
  EXPECT_EQ(qos_to_msg(rclcpp::QoS(rclcpp::KeepAll())).history, QoSMsg::HISTORY_KEEP_ALL);
  EXPECT_EQ(qos_to_msg(rclcpp::QoS(1).history(rclcpp::HistoryPolicy::Unknown)).history, QoSMsg::HISTORY_UNKNOWN);
}

TEST(QoSMapping, Reliability)
{
  EXPECT_EQ(
    qos_to_msg(rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::SystemDefault)).reliability,
    QoSMsg::RELIABILITY_SYSTEM_DEFAULT);
  EXPECT_EQ(qos_to_msg(rclcpp::QoS(1).reliable()).reliability, QoSMsg::RELIABILITY_RELIABLE);
  EXPECT_EQ(qos_to_msg(rclcpp::QoS(1).best_effort()).reliability, QoSMsg::RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(
    qos_to_msg(rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::Unknown)).reliability,
    QoSMsg::RELIABILITY_UNKNOWN);
#ifndef ROS2_HUMBLE
  EXPECT_EQ(
    qos_to_msg(rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::BestAvailable)).reliability,
    QoSMsg::RELIABILITY_BEST_AVAILABLE);
#endif
}

TEST(QoSMapping, Durability)
{
  EXPECT_EQ(
    qos_to_msg(rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::SystemDefault)).durability,
    QoSMsg::DURABILITY_SYSTEM_DEFAULT);
  EXPECT_EQ(qos_to_msg(rclcpp::QoS(1).transient_local()).durability, QoSMsg::DURABILITY_TRANSIENT_LOCAL);
  EXPECT_EQ(qos_to_msg(rclcpp::QoS(1).durability_volatile()).durability, QoSMsg::DURABILITY_VOLATILE);
  EXPECT_EQ(
    qos_to_msg(rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::Unknown)).durability, QoSMsg::DURABILITY_UNKNOWN);
#ifndef ROS2_HUMBLE
  EXPECT_EQ(
    qos_to_msg(rclcpp::QoS(1).durability(rclcpp::DurabilityPolicy::BestAvailable)).durability,
    QoSMsg::DURABILITY_BEST_AVAILABLE);
#endif
}

TEST(QoSMapping, Liveliness)
{
  EXPECT_EQ(
    qos_to_msg(rclcpp::QoS(1).liveliness(rclcpp::LivelinessPolicy::SystemDefault)).liveliness,
    QoSMsg::LIVELINESS_SYSTEM_DEFAULT);
  EXPECT_EQ(
    qos_to_msg(rclcpp::QoS(1).liveliness(rclcpp::LivelinessPolicy::Automatic)).liveliness,
    QoSMsg::LIVELINESS_AUTOMATIC);
  EXPECT_EQ(
    qos_to_msg(rclcpp::QoS(1).liveliness(rclcpp::LivelinessPolicy::ManualByTopic)).liveliness,
    QoSMsg::LIVELINESS_MANUAL_BY_TOPIC);
  EXPECT_EQ(
    qos_to_msg(rclcpp::QoS(1).liveliness(rclcpp::LivelinessPolicy::Unknown)).liveliness, QoSMsg::LIVELINESS_UNKNOWN);
#ifndef ROS2_HUMBLE
  EXPECT_EQ(
    qos_to_msg(rclcpp::QoS(1).liveliness(rclcpp::LivelinessPolicy::BestAvailable)).liveliness,
    QoSMsg::LIVELINESS_BEST_AVAILABLE);
#endif
}

TEST(QoSMapping, DepthAndDurationsCarried)
{
  rclcpp::QoS qos(7);
  qos.deadline(rclcpp::Duration(1, 500));
  qos.lifespan(rclcpp::Duration(2, 0));
  qos.liveliness_lease_duration(rclcpp::Duration(3, 4));

  const auto msg = qos_to_msg(qos);
  EXPECT_EQ(msg.depth, 7u);
  EXPECT_EQ(msg.deadline.sec, 1);
  EXPECT_EQ(msg.deadline.nanosec, 500u);
  EXPECT_EQ(msg.lifespan.sec, 2);
  EXPECT_EQ(msg.lifespan.nanosec, 0u);
  EXPECT_EQ(msg.liveliness_lease_duration.sec, 3);
  EXPECT_EQ(msg.liveliness_lease_duration.nanosec, 4u);
}

TEST(QoSMapping, ObservedQoSNeverSystemDefaultForExplicit)
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable().durability_volatile().liveliness(rclcpp::LivelinessPolicy::Automatic);
  const auto msg = qos_to_msg(qos);
  EXPECT_NE(msg.reliability, QoSMsg::RELIABILITY_SYSTEM_DEFAULT);
  EXPECT_NE(msg.durability, QoSMsg::DURABILITY_SYSTEM_DEFAULT);
#ifndef ROS2_HUMBLE
  EXPECT_NE(msg.reliability, QoSMsg::RELIABILITY_BEST_AVAILABLE);
  EXPECT_NE(msg.durability, QoSMsg::DURABILITY_BEST_AVAILABLE);
#endif
}

TEST(QoSMapping, UnknownQoSMsgAllUnknown)
{
  const auto msg = unknown_qos_msg();
  EXPECT_EQ(msg.history, QoSMsg::HISTORY_UNKNOWN);
  EXPECT_EQ(msg.reliability, QoSMsg::RELIABILITY_UNKNOWN);
  EXPECT_EQ(msg.durability, QoSMsg::DURABILITY_UNKNOWN);
  EXPECT_EQ(msg.liveliness, QoSMsg::LIVELINESS_UNKNOWN);
}

TEST(QoSMapping, InfiniteDeadlineClampedToInt32Max)
{
  // The default deadline is the rmw "infinite/unspecified" sentinel, which must
  // canonicalise to {INT32_MAX, 0} (CDR-valid; differs from graph-monitor's 0,0).
  const auto msg = qos_to_msg(rclcpp::QoS(1));
  EXPECT_EQ(msg.deadline.sec, 2147483647);
  EXPECT_EQ(msg.deadline.nanosec, 0u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
