// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// The `observe` executable: a thin node wrapper around observe_node that does
// the observation + latched publish on /nodl/observed_node.  The CLI contract
// here is depended on by the integration test and the future `ros2 nodl
// describe` verb (which shells out to this binary) -- do not change it.
//
// Usage:
//   observe <node_fqn> [--timeout SECONDS] [--no-parameters]
//                      [--spin-seconds N] [--topic TOPIC]
// Defaults: timeout 5.0, parameters on, spin-seconds 0 (spin forever until
// SIGINT), topic /nodl/observed_node.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "nodl_observe/observe.hpp"
#include "nodl_observe/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/node.hpp"

namespace
{

void print_usage()
{
  fprintf(
    stderr,
    "Usage: observe <node_fqn> [--timeout SECONDS] [--no-parameters] "
    "[--spin-seconds N] [--topic TOPIC]\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Strip ROS-specific args (e.g. --ros-args ...) before our own parsing.
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  std::string fqn;
  double timeout_sec = 5.0;
  bool include_parameters = true;
  double spin_seconds = 0.0;
  std::string topic = "/nodl/observed_node";

  // args[0] is the program name; the first positional is the node FQN.
  for (size_t i = 1; i < args.size(); ++i) {
    const std::string & a = args[i];
    if (a == "--timeout") {
      if (++i >= args.size()) {
        print_usage();
        rclcpp::shutdown();
        return 2;
      }
      timeout_sec = std::stod(args[i]);
    } else if (a == "--no-parameters") {
      include_parameters = false;
    } else if (a == "--spin-seconds") {
      if (++i >= args.size()) {
        print_usage();
        rclcpp::shutdown();
        return 2;
      }
      spin_seconds = std::stod(args[i]);
    } else if (a == "--topic") {
      if (++i >= args.size()) {
        print_usage();
        rclcpp::shutdown();
        return 2;
      }
      topic = args[i];
    } else if (a == "--help" || a == "-h") {
      print_usage();
      rclcpp::shutdown();
      return 0;
    } else if (!a.empty() && a[0] == '-') {
      fprintf(stderr, "Unknown option: %s\n", a.c_str());
      print_usage();
      rclcpp::shutdown();
      return 2;
    } else if (fqn.empty()) {
      fqn = a;
    } else {
      fprintf(stderr, "Unexpected argument: %s\n", a.c_str());
      print_usage();
      rclcpp::shutdown();
      return 2;
    }
  }

  if (fqn.empty()) {
    print_usage();
    rclcpp::shutdown();
    return 2;
  }

  auto node = std::make_shared<rclcpp::Node>("nodl_observe");

  nodl_observe::Options opts;
  opts.timeout = std::chrono::duration<double>(timeout_sec);
  opts.include_parameters = include_parameters;

  rosgraph_msgs::msg::Node msg;
  try {
    msg = nodl_observe::observe_node(*node, fqn, opts);
  } catch (const nodl_observe::NodeNotFoundError & e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Latched publish so transient_local subscribers can fetch the observation
  // after the fact.
  auto publisher = node->create_publisher<rosgraph_msgs::msg::Node>(topic, nodl_observe::latched_qos());
  publisher->publish(msg);

  // Keep the process alive so late subscribers can still pull the latched sample.
  if (spin_seconds <= 0.0) {
    rclcpp::spin(node);
  } else {
    // Spin for a bounded wall-clock window.  (A broken-promise future would be
    // reported "ready" immediately and not wait at all, so loop on the clock.)
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                          std::chrono::duration<double>(spin_seconds));
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
      exec.spin_once(std::chrono::milliseconds(50));
    }
  }

  rclcpp::shutdown();
  return 0;
}
