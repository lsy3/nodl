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
#include <memory>
#include <string>
#include <vector>

#include "CLI/CLI.hpp"
#include "nodl_observe/observe.hpp"
#include "nodl_observe/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Strip ROS-specific args (e.g. --ros-args ...) before our own parsing.
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  std::string fqn;
  double timeout_sec = 5.0;
  bool no_parameters = false;
  double spin_seconds = 0.0;
  std::string topic = "/nodl/observed_node";

  CLI::App app{"observe"};
  app.add_option("node_fqn", fqn, "Fully-qualified name of the node to observe")->required();
  app.add_option("--timeout", timeout_sec, "Observation timeout in seconds");
  app.add_flag("--no-parameters", no_parameters, "Do not collect parameters");
  app.add_option("--spin-seconds", spin_seconds, "Seconds to keep the process alive after publishing");
  app.add_option("--topic", topic, "Topic to publish the observation on");

  // Feed the ROS-stripped args to CLI11 via the argc/argv overload (argv[0] is
  // the program name).
  std::vector<const char *> cli_argv;
  cli_argv.reserve(args.size());
  for (const std::string & a : args) {
    cli_argv.push_back(a.c_str());
  }
  try {
    app.parse(static_cast<int>(cli_argv.size()), cli_argv.data());
  } catch (const CLI::ParseError & e) {
    const int code = app.exit(e);
    rclcpp::shutdown();
    // Map CLI11's success exit (--help) to 0 and any parse failure to 2, matching
    // the previous hand-rolled contract.
    return code == 0 ? 0 : 2;
  }

  const auto node = std::make_shared<rclcpp::Node>("nodl_observe");

  nodl_observe::Options opts;
  opts.timeout = std::chrono::duration<double>(timeout_sec);
  opts.include_parameters = !no_parameters;

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
  const auto publisher = node->create_publisher<rosgraph_msgs::msg::Node>(topic, nodl_observe::latched_qos());
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
