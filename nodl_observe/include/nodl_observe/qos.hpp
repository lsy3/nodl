// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef NODL_OBSERVE__QOS_HPP_
#define NODL_OBSERVE__QOS_HPP_

#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"
#include "rosgraph_msgs/msg/qo_s_profile.hpp"

namespace nodl_observe
{

/// Convert an rclcpp::Duration to a builtin_interfaces/Duration, canonicalising
/// the rmw "infinite"/"unspecified" sentinels (and any int32-overflowing value)
/// to a fixed, CDR-valid sentinel of {sec = INT32_MAX, nanosec = 0}.
///
/// This DIFFERS from graph-monitor's convert_maybe_inifite_durations (which uses
/// {0, 0}); the plan (#2) decided on INT32_MAX so the value is cross-distro
/// identical and serialization-safe for MCAP.  Applied uniformly on every distro.
builtin_interfaces::msg::Duration to_duration_msg(const rclcpp::Duration & d);

/// Translate an rclcpp::QoS into a rosgraph_msgs/QoSProfile message.
///
/// Maps each policy via an explicit switch so an upstream enum drift surfaces as
/// a -Werror=switch build failure (the C++ analogue of the Python KeyError).  An
/// *observed* policy can never legitimately be SYSTEM_DEFAULT or BEST_AVAILABLE
/// -- those are request-time placeholders -- but they are still mapped so a stray
/// one shows up faithfully rather than crashing.
rosgraph_msgs::msg::QoSProfile qos_to_msg(const rclcpp::QoS & qos);

/// The QoS profile of the latched observation publish: reliable +
/// transient_local + keep_last(1).  The contract for /nodl/observed_node; lives
/// here so publishers and subscribers of the latched topic share one definition.
rclcpp::QoS latched_qos();

/// A QoSProfile message whose policies are all *_UNKNOWN, with durations/depth
/// left at zero.  Used for service / action-service endpoints, whose actual QoS
/// is not observable (there is no get_*_info_by_service API).
rosgraph_msgs::msg::QoSProfile unknown_qos_msg();

}  // namespace nodl_observe

#endif  // NODL_OBSERVE__QOS_HPP_
