// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef NODL_OBSERVE__PARAMETERS_HPP_
#define NODL_OBSERVE__PARAMETERS_HPP_

#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"

namespace nodl_observe
{

using ParameterPair =
  std::pair<std::vector<rcl_interfaces::msg::ParameterDescriptor>, std::vector<rcl_interfaces::msg::ParameterValue>>;

/// Pair descriptors with values into two parallel, sorted-by-name lists.  Pure:
/// `descriptors` carry their own name; `values` are positional, aligned to
/// `names` (the listed order).  Only names present in BOTH a descriptor and a
/// value are kept (robust to a length mismatch from a parameter removed
/// mid-observation).
ParameterPair build_parameters(
  const std::vector<std::string> & names,
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & descriptors,
  const std::vector<rcl_interfaces::msg::ParameterValue> & values);

/// Collect parameter descriptors and current values from the target node via
/// its ~/list/describe/get_parameters services, using an AsyncParametersClient
/// driven by `executor`.  Returns ([], []) -- with a logged warning -- if the
/// target is unresponsive or exposes no parameters; never throws (graceful
/// degradation).  `timeout` is a shared ceiling across all round-trips.
ParameterPair collect_parameters(
  rclcpp::Node & node,
  rclcpp::Executor & executor,
  const std::string & target_fqn,
  std::chrono::duration<double> timeout);

}  // namespace nodl_observe

#endif  // NODL_OBSERVE__PARAMETERS_HPP_
