// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
#include "nodl_observe/observe.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "nodl_observe/actions.hpp"
#include "nodl_observe/endpoints.hpp"
#include "nodl_observe/parameters.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"

namespace nodl_observe
{

namespace
{

using Clock = std::chrono::steady_clock;
using NamesAndTypes = std::map<std::string, std::vector<std::string>>;

// Graph stability poll parameters.  No "discovery complete" signal exists, so we
// poll until the target's endpoint set is unchanged across this many consecutive
// samples, bounded by the overall deadline.
constexpr int kStablePolls = 3;
constexpr std::chrono::milliseconds kPollInterval{200};

std::string strip_trailing_slash(const std::string & s)
{
  if (s.size() > 1 && s.back() == '/') {
    return s.substr(0, s.size() - 1);
  }
  return s;
}

// One snapshot of the target's four by-node endpoint queries.
struct Snapshot
{
  NamesAndTypes pubs;
  NamesAndTypes subs;
  NamesAndTypes srv_servers;
  NamesAndTypes srv_clients;

  bool operator==(const Snapshot & o) const
  {
    return std::tie(pubs, subs, srv_servers, srv_clients) == std::tie(o.pubs, o.subs, o.srv_servers, o.srv_clients);
  }
};

Snapshot endpoint_snapshot(
  rclcpp::node_interfaces::NodeGraphInterface & graph, const std::string & name, const std::string & ns)
{
  Snapshot s;
  s.pubs = graph.get_publisher_names_and_types_by_node(name, ns, false);
  s.subs = graph.get_subscriber_names_and_types_by_node(name, ns, false);
  s.srv_servers = graph.get_service_names_and_types_by_node(name, ns);
  s.srv_clients = graph.get_client_names_and_types_by_node(name, ns);
  return s;
}

std::pair<std::string, std::string> wait_for_node(
  rclcpp::node_interfaces::NodeGraphInterface & graph,
  const std::string & target_fqn,
  const Clock::time_point & deadline)
{
  const auto [name, ns] = split_fqn(target_fqn);
  const std::string target_ns = strip_trailing_slash(ns);
  while (true) {
    for (const auto & [n, n_ns] : graph.get_node_names_and_namespaces()) {
      if (n == name && strip_trailing_slash(n_ns) == target_ns) {
        return {name, ns};
      }
    }
    if (Clock::now() >= deadline) {
      throw NodeNotFoundError("Node '" + target_fqn + "' did not appear in the graph within the timeout.");
    }
    std::this_thread::sleep_for(kPollInterval);
  }
}

Snapshot wait_for_stable_graph(
  rclcpp::node_interfaces::NodeGraphInterface & graph,
  const std::string & name,
  const std::string & ns,
  const Clock::time_point & deadline)
{
  Snapshot snapshot = endpoint_snapshot(graph, name, ns);
  Snapshot previous = snapshot;
  int stable_count = 1;
  while (stable_count < kStablePolls && Clock::now() < deadline) {
    std::this_thread::sleep_for(kPollInterval);
    snapshot = endpoint_snapshot(graph, name, ns);
    if (snapshot == previous) {
      ++stable_count;
    } else {
      stable_count = 1;
      previous = snapshot;
    }
  }
  return snapshot;
}

// Group TopicEndpointInfo objects by topic, keeping only this node's endpoints
// (matched on node name + trailing-slash-stripped namespace).
std::map<std::string, std::vector<rclcpp::TopicEndpointInfo>> filter_infos_to_node(
  const std::map<std::string, std::vector<rclcpp::TopicEndpointInfo>> & infos,
  const std::string & name,
  const std::string & ns)
{
  const std::string target_ns = strip_trailing_slash(ns);
  std::map<std::string, std::vector<rclcpp::TopicEndpointInfo>> out;
  for (const auto & [topic_name, info_list] : infos) {
    std::vector<rclcpp::TopicEndpointInfo> matched;
    for (const auto & info : info_list) {
      if (info.node_name() == name && strip_trailing_slash(info.node_namespace()) == target_ns) {
        matched.push_back(info);
      }
    }
    if (!matched.empty()) {
      out.emplace(topic_name, std::move(matched));
    }
  }
  return out;
}

void collect_endpoints(
  rclcpp::Node & node,
  const std::string & name,
  const std::string & ns,
  rosgraph_msgs::msg::Node & msg,
  const Snapshot & snapshot)
{
  auto graph = node.get_node_graph_interface();

  std::map<std::string, std::vector<rclcpp::TopicEndpointInfo>> pub_infos_raw;
  for (const auto & [topic_name, types] : snapshot.pubs) {
    (void)types;
    pub_infos_raw.emplace(topic_name, graph->get_publishers_info_by_topic(topic_name));
  }
  std::map<std::string, std::vector<rclcpp::TopicEndpointInfo>> sub_infos_raw;
  for (const auto & [topic_name, types] : snapshot.subs) {
    (void)types;
    sub_infos_raw.emplace(topic_name, graph->get_subscriptions_info_by_topic(topic_name));
  }

  const auto pub_infos = filter_infos_to_node(pub_infos_raw, name, ns);
  const auto sub_infos = filter_infos_to_node(sub_infos_raw, name, ns);

  auto publishers = build_topic_endpoints(snapshot.pubs, pub_infos);
  auto subscriptions = build_topic_endpoints(snapshot.subs, sub_infos);
  auto service_servers = build_service_endpoints(snapshot.srv_servers);
  auto service_clients = build_service_endpoints(snapshot.srv_clients);

  const auto action_servers_nt = get_action_server_names_and_types_by_node(node, name, ns);
  const auto action_clients_nt = get_action_client_names_and_types_by_node(node, name, ns);

  // Action servers own server-side services + the feedback/status publishers;
  // action clients own client-side services + the feedback/status subscriptions.
  auto action_servers = fold_actions(action_servers_nt, service_servers, publishers);
  auto action_clients = fold_actions(action_clients_nt, service_clients, subscriptions);

  msg.publishers = std::move(publishers);
  msg.subscriptions = std::move(subscriptions);
  msg.service_servers = std::move(service_servers);
  msg.service_clients = std::move(service_clients);
  msg.action_servers = std::move(action_servers);
  msg.action_clients = std::move(action_clients);
}

}  // namespace

std::pair<std::string, std::string> split_fqn(const std::string & target_fqn)
{
  std::string stripped = strip_trailing_slash(target_fqn);
  const auto pos = stripped.rfind('/');
  if (pos == std::string::npos) {
    return {stripped, "/"};
  }
  std::string name = stripped.substr(pos + 1);
  std::string ns = stripped.substr(0, pos);
  if (ns.empty()) {
    ns = "/";
  }
  return {name, ns};
}

rosgraph_msgs::msg::Node observe_node(rclcpp::Node & node, const std::string & target_fqn, const Options & opts)
{
  const Clock::time_point deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(opts.timeout);

  auto & graph = *node.get_node_graph_interface();
  const auto [name, ns] = wait_for_node(graph, target_fqn, deadline);
  const Snapshot snapshot = wait_for_stable_graph(graph, name, ns, deadline);

  rosgraph_msgs::msg::Node msg;
  msg.name = target_fqn;

  collect_endpoints(node, name, ns, msg, snapshot);

  if (opts.include_parameters) {
    const auto remaining = std::max(
      std::chrono::duration<double>(0.0),
      std::chrono::duration_cast<std::chrono::duration<double>>(deadline - Clock::now()));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node.get_node_base_interface());
    auto params = collect_parameters(node, executor, target_fqn, remaining);
    executor.remove_node(node.get_node_base_interface());

    msg.parameters = std::move(params.first);
    msg.parameter_values = std::move(params.second);
  }

  return msg;
}

}  // namespace nodl_observe
