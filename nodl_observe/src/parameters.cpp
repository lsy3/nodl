// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
#include "nodl_observe/parameters.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"

namespace nodl_observe
{

namespace
{

using Clock = std::chrono::steady_clock;
using DSeconds = std::chrono::duration<double>;

// Remaining budget from a shared monotonic deadline (mirrors the Python `_call`
// which derives `remaining` from a single time.monotonic()-based deadline).
DSeconds remaining_until(const Clock::time_point & deadline)
{
  return std::chrono::duration_cast<DSeconds>(deadline - Clock::now());
}

}  // namespace

ParameterPair build_parameters(
  const std::vector<std::string> & names,
  const std::vector<rcl_interfaces::msg::ParameterDescriptor> & descriptors,
  const std::vector<rcl_interfaces::msg::ParameterValue> & values)
{
  std::map<std::string, const rcl_interfaces::msg::ParameterDescriptor *> desc_by_name;
  for (const auto & d : descriptors) {
    desc_by_name[d.name] = &d;
  }
  // GetParameters values are positional (no name field), aligned to the request
  // order which mirrors the listed names.
  std::map<std::string, const rcl_interfaces::msg::ParameterValue *> value_by_name;
  const size_t n = std::min(names.size(), values.size());
  for (size_t i = 0; i < n; ++i) {
    value_by_name[names[i]] = &values[i];
  }

  std::vector<std::tuple<std::string, rcl_interfaces::msg::ParameterDescriptor, rcl_interfaces::msg::ParameterValue>>
    paired;
  for (const auto & name : names) {
    auto dit = desc_by_name.find(name);
    auto vit = value_by_name.find(name);
    if (dit == desc_by_name.end() || vit == value_by_name.end()) {
      continue;
    }
    paired.emplace_back(name, *dit->second, *vit->second);
  }

  std::sort(
    paired.begin(), paired.end(), [](const auto & a, const auto & b) { return std::get<0>(a) < std::get<0>(b); });

  ParameterPair out;
  out.first.reserve(paired.size());
  out.second.reserve(paired.size());
  for (auto & item : paired) {
    out.first.push_back(std::move(std::get<1>(item)));
    out.second.push_back(std::move(std::get<2>(item)));
  }
  return out;
}

ParameterPair collect_parameters(
  rclcpp::Node & node,
  rclcpp::Executor & executor,
  const std::string & target_fqn,
  std::chrono::duration<double> timeout)
{
  rclcpp::Logger logger = node.get_logger();
  const Clock::time_point deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(timeout);

  try {
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(
      node.get_node_base_interface(),
      node.get_node_topics_interface(),
      node.get_node_graph_interface(),
      node.get_node_services_interface(),
      target_fqn);

    DSeconds remaining = remaining_until(deadline);
    if (remaining.count() <= 0.0 || !client->wait_for_service(remaining)) {
      RCLCPP_WARN(
        logger, "Could not reach parameter services on '%s'; reporting empty parameters.", target_fqn.c_str());
      return ParameterPair{};
    }

    // list_parameters({}, 0): all parameters, full depth.
    auto list_future = client->list_parameters({}, 0);
    remaining = remaining_until(deadline);
    if (
      remaining.count() <= 0.0 ||
      executor.spin_until_future_complete(list_future, remaining) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(
        logger, "Could not reach parameter services on '%s'; reporting empty parameters.", target_fqn.c_str());
      return ParameterPair{};
    }

    const rcl_interfaces::msg::ListParametersResult list_result = list_future.get();
    const std::vector<std::string> names = list_result.names;
    if (names.empty()) {
      return ParameterPair{};
    }

    auto describe_future = client->describe_parameters(names);
    remaining = remaining_until(deadline);
    bool describe_ok = remaining.count() > 0.0 && executor.spin_until_future_complete(describe_future, remaining) ==
                                                    rclcpp::FutureReturnCode::SUCCESS;

    auto get_future = client->get_parameters(names);
    remaining = remaining_until(deadline);
    bool get_ok = remaining.count() > 0.0 &&
                  executor.spin_until_future_complete(get_future, remaining) == rclcpp::FutureReturnCode::SUCCESS;

    if (!describe_ok || !get_ok) {
      RCLCPP_WARN(
        logger,
        "Listed parameters on '%s' but could not describe or read them; "
        "reporting empty parameters.",
        target_fqn.c_str());
      return ParameterPair{};
    }

    const std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors = describe_future.get();

    // get_parameters yields rclcpp::Parameter objects; their value messages are
    // positionally aligned to `names` (the request order).
    const std::vector<rclcpp::Parameter> params = get_future.get();
    std::vector<rcl_interfaces::msg::ParameterValue> values;
    values.reserve(params.size());
    for (const auto & p : params) {
      values.push_back(p.get_value_message());
    }

    return build_parameters(names, descriptors, values);
  } catch (const std::exception & e) {
    // Graceful-degradation contract: a target that dies or tears its parameter
    // services down mid-observation must degrade to empty arrays, never fail.
    RCLCPP_WARN(
      logger, "Parameter collection on '%s' failed (%s); reporting empty parameters.", target_fqn.c_str(), e.what());
    return ParameterPair{};
  }
}

}  // namespace nodl_observe
