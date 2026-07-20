// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
#include "nodl_observe/qos.hpp"

#include <cstdint>

#include "rmw/time.h"

namespace nodl_observe
{

builtin_interfaces::msg::Duration to_duration_msg(const rclcpp::Duration & d)
{
  builtin_interfaces::msg::Duration msg;
  const rmw_time_t t = d.to_rmw_time();
  // Canonicalise the rmw infinite / unspecified sentinels, and any value that
  // overflows the int32 `sec` field, to a fixed CDR-valid sentinel.  (rmw_time_t
  // `sec` is uint64, so an "infinite" deadline of ~9223372036 s would otherwise
  // truncate / not round-trip CDR.)
  if (
    rmw_time_equal(t, RMW_DURATION_INFINITE) || rmw_time_equal(t, RMW_DURATION_UNSPECIFIED) ||
    t.sec > static_cast<uint64_t>(INT32_MAX))
  {
    msg.sec = INT32_MAX;
    msg.nanosec = 0;
  } else {
    msg.sec = static_cast<int32_t>(t.sec);
    msg.nanosec = static_cast<uint32_t>(t.nsec);
  }
  return msg;
}

rosgraph_msgs::msg::QoSProfile qos_to_msg(const rclcpp::QoS & qos)
{
  using QoSMsg = rosgraph_msgs::msg::QoSProfile;
  QoSMsg msg;

  msg.depth = qos.depth();

  // Explicit switches (not a static_cast of the raw enum) so a divergence
  // between the rclcpp enums and the QoSProfile.msg constants is a build-time
  // -Werror=switch failure rather than a silently wrong byte.  default: is
  // intentionally omitted on each switch for the same reason.
  switch (qos.history()) {
    case rclcpp::HistoryPolicy::SystemDefault:
      msg.history = QoSMsg::HISTORY_SYSTEM_DEFAULT;
      break;
    case rclcpp::HistoryPolicy::KeepLast:
      msg.history = QoSMsg::HISTORY_KEEP_LAST;
      break;
    case rclcpp::HistoryPolicy::KeepAll:
      msg.history = QoSMsg::HISTORY_KEEP_ALL;
      break;
    case rclcpp::HistoryPolicy::Unknown:
      msg.history = QoSMsg::HISTORY_UNKNOWN;
      break;
  }

  switch (qos.reliability()) {
    case rclcpp::ReliabilityPolicy::SystemDefault:
      msg.reliability = QoSMsg::RELIABILITY_SYSTEM_DEFAULT;
      break;
    case rclcpp::ReliabilityPolicy::Reliable:
      msg.reliability = QoSMsg::RELIABILITY_RELIABLE;
      break;
    case rclcpp::ReliabilityPolicy::BestEffort:
      msg.reliability = QoSMsg::RELIABILITY_BEST_EFFORT;
      break;
    case rclcpp::ReliabilityPolicy::Unknown:
      msg.reliability = QoSMsg::RELIABILITY_UNKNOWN;
      break;
#ifndef ROS2_HUMBLE
    case rclcpp::ReliabilityPolicy::BestAvailable:
      msg.reliability = QoSMsg::RELIABILITY_BEST_AVAILABLE;
      break;
#endif
  }

  switch (qos.durability()) {
    case rclcpp::DurabilityPolicy::SystemDefault:
      msg.durability = QoSMsg::DURABILITY_SYSTEM_DEFAULT;
      break;
    case rclcpp::DurabilityPolicy::TransientLocal:
      msg.durability = QoSMsg::DURABILITY_TRANSIENT_LOCAL;
      break;
    case rclcpp::DurabilityPolicy::Volatile:
      msg.durability = QoSMsg::DURABILITY_VOLATILE;
      break;
    case rclcpp::DurabilityPolicy::Unknown:
      msg.durability = QoSMsg::DURABILITY_UNKNOWN;
      break;
#ifndef ROS2_HUMBLE
    case rclcpp::DurabilityPolicy::BestAvailable:
      msg.durability = QoSMsg::DURABILITY_BEST_AVAILABLE;
      break;
#endif
  }

  switch (qos.liveliness()) {
    case rclcpp::LivelinessPolicy::SystemDefault:
      msg.liveliness = QoSMsg::LIVELINESS_SYSTEM_DEFAULT;
      break;
    case rclcpp::LivelinessPolicy::Automatic:
      msg.liveliness = QoSMsg::LIVELINESS_AUTOMATIC;
      break;
    case rclcpp::LivelinessPolicy::ManualByTopic:
      msg.liveliness = QoSMsg::LIVELINESS_MANUAL_BY_TOPIC;
      break;
    case rclcpp::LivelinessPolicy::Unknown:
      msg.liveliness = QoSMsg::LIVELINESS_UNKNOWN;
      break;
#ifndef ROS2_HUMBLE
    case rclcpp::LivelinessPolicy::BestAvailable:
      msg.liveliness = QoSMsg::LIVELINESS_BEST_AVAILABLE;
      break;
#endif
  }

  msg.deadline = to_duration_msg(qos.deadline());
  msg.lifespan = to_duration_msg(qos.lifespan());
  msg.liveliness_lease_duration = to_duration_msg(qos.liveliness_lease_duration());

  return msg;
}

rclcpp::QoS latched_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

rosgraph_msgs::msg::QoSProfile unknown_qos_msg()
{
  using QoSMsg = rosgraph_msgs::msg::QoSProfile;
  QoSMsg msg;
  msg.history = QoSMsg::HISTORY_UNKNOWN;
  msg.reliability = QoSMsg::RELIABILITY_UNKNOWN;
  msg.durability = QoSMsg::DURABILITY_UNKNOWN;
  msg.liveliness = QoSMsg::LIVELINESS_UNKNOWN;
  // Durations and depth stay at their message defaults (zero) -- unobserved.
  return msg;
}

}  // namespace nodl_observe
