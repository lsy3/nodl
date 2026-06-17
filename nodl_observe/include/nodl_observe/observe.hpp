// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef NODL_OBSERVE__OBSERVE_HPP_
#define NODL_OBSERVE__OBSERVE_HPP_

#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/node.hpp"
#include "rosgraph_msgs/msg/node.hpp"

namespace nodl_observe
{

/// Observation options.  `timeout` is a ceiling, not a fixed duration; it bounds
/// both the discovery/stability wait and the parameter round-trips.
struct Options
{
  std::chrono::duration<double> timeout{5.0};
  bool include_parameters{true};
};

/// The target node never appeared in the graph within the timeout.
class NodeNotFoundError : public std::runtime_error
{
public:
  explicit NodeNotFoundError(const std::string & msg)
  : std::runtime_error(msg)
  {}
};

/// Observe a running node and return its runtime rosgraph_msgs/Node message.
///
/// Uses the caller-provided node for all graph queries and parameter service
/// calls; never creates its own node.  Parameter collection drives async futures
/// via a short-lived internal SingleThreadedExecutor, so the caller MUST NOT be
/// spinning `node` on another thread concurrently.
///
/// All endpoint arrays come back sorted (by name, then type) for deterministic
/// output.  Throws NodeNotFoundError if the target never appears within
/// `opts.timeout`.
rosgraph_msgs::msg::Node observe_node(rclcpp::Node & node, const std::string & target_fqn, const Options & opts = {});

/// Split a fully-qualified node name into (name, namespace).
/// `/ns/sub/talker` -> ("talker", "/ns/sub"); `/talker` -> ("talker", "/").
/// Exposed for unit testing.
std::pair<std::string, std::string> split_fqn(const std::string & target_fqn);

}  // namespace nodl_observe

#endif  // NODL_OBSERVE__OBSERVE_HPP_
