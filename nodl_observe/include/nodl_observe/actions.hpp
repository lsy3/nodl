// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef NODL_OBSERVE__ACTIONS_HPP_
#define NODL_OBSERVE__ACTIONS_HPP_

#include <map>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"

namespace nodl_observe
{

/// Action server graph query, by node.  Wraps rcl_action_get_server_names_and
/// _types_by_node (there is no rclcpp_action wrapper -- the one real porting
/// gap).  Returns action *base* names (not _action/* constituents), matching
/// rclpy.action.graph.  On any rcl error, throws std::runtime_error.
std::map<std::string, std::vector<std::string>> get_action_server_names_and_types_by_node(
  rclcpp::Node & node, const std::string & name, const std::string & ns);

/// Action client graph query, by node.  As above, via
/// rcl_action_get_client_names_and_types_by_node.
std::map<std::string, std::vector<std::string>> get_action_client_names_and_types_by_node(
  rclcpp::Node & node, const std::string & name, const std::string & ns);

}  // namespace nodl_observe

#endif  // NODL_OBSERVE__ACTIONS_HPP_
