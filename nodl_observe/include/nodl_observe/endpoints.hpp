// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef NODL_OBSERVE__ENDPOINTS_HPP_
#define NODL_OBSERVE__ENDPOINTS_HPP_

#include <map>
#include <string>
#include <vector>

#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rosgraph_msgs/msg/action.hpp"
#include "rosgraph_msgs/msg/interface_type.hpp"
#include "rosgraph_msgs/msg/service.hpp"
#include "rosgraph_msgs/msg/topic.hpp"
#include "rosgraph_msgs/msg/type_hash.hpp"
#ifndef ROS2_HUMBLE
  #include "rosidl_runtime_c/type_hash.h"  // REP-2011 topic type hashes (Iron+ only)
#endif

namespace nodl_observe
{

/// Pure builders turning raw graph-query results into Node sub-messages.
/// Nothing here touches the ROS graph: every function takes already-collected
/// plain data and returns filled messages, so the endpoint-collection layer is
/// unit-testable without an executor.

#ifndef ROS2_HUMBLE
/// Copy a rosidl_type_hash_t into a rosgraph_msgs/TypeHash message (Iron+ only;
/// REP-2011 type hashes do not exist pre-Iron).  Up to 32 bytes are copied.
rosgraph_msgs::msg::TypeHash type_hash_msg(const rosidl_type_hash_t & src);

/// Build an InterfaceType from a type name and a type hash (Iron+ only).
rosgraph_msgs::msg::InterfaceType interface_type(const std::string & name, const rosidl_type_hash_t & src);
#endif

/// Build an InterfaceType from a type name only; leaves the hash at the message
/// default (version 1, all-zero value) -- the honest-unknown state, and the only
/// form available on Humble (which has no REP-2011 type hash).
rosgraph_msgs::msg::InterfaceType interface_type(const std::string & name);

/// Build a Topic from name/type/QoS with no type hash.  The unit-testable core
/// (tests need not construct a live rclcpp::TopicEndpointInfo, whose public ctors
/// are impractical to fixture) and the only form on Humble.
rosgraph_msgs::msg::Topic build_topic(const std::string & name, const std::string & type_name, const rclcpp::QoS & qos);

#ifndef ROS2_HUMBLE
/// Build a Topic from raw fields including a REP-2011 type hash (Iron+ only).
/// A null hash falls back to the no-hash overload.
rosgraph_msgs::msg::Topic build_topic(
  const std::string & name, const std::string & type_name, const rclcpp::QoS & qos, const rosidl_type_hash_t * hash);
#endif

/// Build a Topic from a TopicEndpointInfo.  On Iron+ the REP-2011 type hash is
/// carried; on Humble (no topic_type_hash member) the hash is left unset.
rosgraph_msgs::msg::Topic build_topic(const std::string & name, const rclcpp::TopicEndpointInfo & info);

/// Build a Service with UNKNOWN QoS and no type hash (neither is observable).
rosgraph_msgs::msg::Service build_service(
  const std::string & name, const std::string & request_type, const std::string & response_type);

/// Build a sorted (by name, then type name) list of Topic messages for one
/// endpoint direction.  If `infos_by_topic` has entries for a topic, one Topic
/// per info; otherwise one name/type-only Topic per declared type (QoS/hash at
/// message defaults) so the endpoint is never dropped.
std::vector<rosgraph_msgs::msg::Topic> build_topic_endpoints(
  const std::map<std::string, std::vector<std::string>> & names_and_types,
  const std::map<std::string, std::vector<rclcpp::TopicEndpointInfo>> & infos_by_topic);

/// Build a sorted (by name, then request type name) list of Service messages.
/// The single reported service type becomes both request and response type.
std::vector<rosgraph_msgs::msg::Service> build_service_endpoints(
  const std::map<std::string, std::vector<std::string>> & names_and_types);

/// Fold hidden <action>/_action/* constituents into Action messages.  Matched
/// constituents are *moved* out of the flat service/topic vectors (erased in
/// place); an _action/* entity with no parent action in the graph is left flat
/// (never discarded); missing constituents get placeholders so the Action stays
/// well-formed.  Returns a sorted (by name, then send_goal request type) list.
std::vector<rosgraph_msgs::msg::Action> fold_actions(
  const std::map<std::string, std::vector<std::string>> & action_names_and_types,
  std::vector<rosgraph_msgs::msg::Service> & service_endpoints,
  std::vector<rosgraph_msgs::msg::Topic> & topic_endpoints);

}  // namespace nodl_observe

#endif  // NODL_OBSERVE__ENDPOINTS_HPP_
