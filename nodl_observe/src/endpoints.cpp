// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
#include "nodl_observe/endpoints.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "nodl_observe/qos.hpp"

namespace nodl_observe
{

namespace
{

// Constituent suffixes of a hidden <action>/_action/* entity, by kind.
constexpr const char * kActionInfix = "/_action/";
const std::array<std::string, 3> kActionServiceSuffixes{"send_goal", "get_result", "cancel_goal"};
const std::array<std::string, 2> kActionTopicSuffixes{"feedback", "status"};

void sort_topics(std::vector<rosgraph_msgs::msg::Topic> & topics)
{
  std::sort(topics.begin(), topics.end(), [](const rosgraph_msgs::msg::Topic & a, const rosgraph_msgs::msg::Topic & b) {
    return std::tie(a.name, a.type.name) < std::tie(b.name, b.type.name);
  });
}

void sort_services(std::vector<rosgraph_msgs::msg::Service> & services)
{
  std::sort(
    services.begin(), services.end(), [](const rosgraph_msgs::msg::Service & a, const rosgraph_msgs::msg::Service & b) {
      return std::tie(a.name, a.request_type.name) < std::tie(b.name, b.request_type.name);
    });
}

void sort_actions(std::vector<rosgraph_msgs::msg::Action> & actions)
{
  std::sort(
    actions.begin(), actions.end(), [](const rosgraph_msgs::msg::Action & a, const rosgraph_msgs::msg::Action & b) {
      return std::tie(a.name, a.send_goal.request_type.name) < std::tie(b.name, b.send_goal.request_type.name);
    });
}

// (request_type, response_type) for a service/action endpoint.  Graph queries
// report a single service type string; Service.msg splits it into request and
// response InterfaceTypes carrying the same name.  First type wins on collision.
std::string first_type(const std::vector<std::string> & types)
{
  return types.empty() ? std::string() : types.front();
}

}  // namespace

#ifndef ROS2_HUMBLE
rosgraph_msgs::msg::TypeHash type_hash_msg(const rosidl_type_hash_t & src)
{
  rosgraph_msgs::msg::TypeHash msg;
  // version is a uint8_t in rosidl_type_hash_t; it is always 0..255, so it maps
  // directly.  (The Python clamped a -1 "unset" sentinel; the C struct has no
  // such negative state -- an unset hash is simply never passed here.)
  msg.version = src.version;
  const size_t n = std::min<size_t>(ROSIDL_TYPE_HASH_SIZE, msg.value.size());
  for (size_t i = 0; i < n; ++i) {
    msg.value[i] = src.value[i];
  }
  return msg;
}

rosgraph_msgs::msg::InterfaceType interface_type(const std::string & name, const rosidl_type_hash_t & src)
{
  rosgraph_msgs::msg::InterfaceType iface;
  iface.name = name;
  iface.hash = type_hash_msg(src);
  return iface;
}
#endif  // ROS2_HUMBLE

rosgraph_msgs::msg::InterfaceType interface_type(const std::string & name)
{
  rosgraph_msgs::msg::InterfaceType iface;
  iface.name = name;
  // hash left at message default (version 1, all-zero value).
  return iface;
}

rosgraph_msgs::msg::Topic build_topic(const std::string & name, const std::string & type_name, const rclcpp::QoS & qos)
{
  rosgraph_msgs::msg::Topic topic;
  topic.name = name;
  topic.type = interface_type(type_name);
  topic.qos = qos_to_msg(qos);
  return topic;
}

#ifndef ROS2_HUMBLE
rosgraph_msgs::msg::Topic build_topic(
  const std::string & name, const std::string & type_name, const rclcpp::QoS & qos, const rosidl_type_hash_t * hash)
{
  if (hash == nullptr) {
    return build_topic(name, type_name, qos);
  }
  rosgraph_msgs::msg::Topic topic;
  topic.name = name;
  topic.type = interface_type(type_name, *hash);
  topic.qos = qos_to_msg(qos);
  return topic;
}
#endif  // ROS2_HUMBLE

rosgraph_msgs::msg::Topic build_topic(const std::string & name, const rclcpp::TopicEndpointInfo & info)
{
#ifdef ROS2_HUMBLE
  // Humble's TopicEndpointInfo has no topic_type_hash() -- leave the hash unset
  // (message default), the same honest-unknown treatment service hashes get.
  return build_topic(name, info.topic_type(), info.qos_profile());
#else
  const rosidl_type_hash_t & hash = info.topic_type_hash();
  return build_topic(name, info.topic_type(), info.qos_profile(), &hash);
#endif
}

rosgraph_msgs::msg::Service build_service(
  const std::string & name, const std::string & request_type, const std::string & response_type)
{
  rosgraph_msgs::msg::Service service;
  service.name = name;
  service.request_type = interface_type(request_type);
  service.response_type = interface_type(response_type);
  service.request_qos = unknown_qos_msg();
  service.response_qos = unknown_qos_msg();
  return service;
}

std::vector<rosgraph_msgs::msg::Topic> build_topic_endpoints(
  const std::map<std::string, std::vector<std::string>> & names_and_types,
  const std::map<std::string, std::vector<rclcpp::TopicEndpointInfo>> & infos_by_topic)
{
  std::vector<rosgraph_msgs::msg::Topic> topics;
  for (const auto & [name, types] : names_and_types) {
    auto it = infos_by_topic.find(name);
    if (it != infos_by_topic.end() && !it->second.empty()) {
      for (const auto & info : it->second) {
        topics.push_back(build_topic(name, info));
      }
    } else {
      // No introspection info -> emit a name/type-only entry per declared type
      // rather than dropping the endpoint; QoS and hash stay at message defaults.
      for (const auto & type_name : types) {
        rosgraph_msgs::msg::Topic topic;
        topic.name = name;
        topic.type = interface_type(type_name);
        topics.push_back(std::move(topic));
      }
    }
  }
  sort_topics(topics);
  return topics;
}

std::vector<rosgraph_msgs::msg::Service> build_service_endpoints(
  const std::map<std::string, std::vector<std::string>> & names_and_types)
{
  std::vector<rosgraph_msgs::msg::Service> services;
  for (const auto & [name, types] : names_and_types) {
    const std::string type_name = first_type(types);
    services.push_back(build_service(name, type_name, type_name));
  }
  sort_services(services);
  return services;
}

std::vector<rosgraph_msgs::msg::Action> fold_actions(
  const std::map<std::string, std::vector<std::string>> & action_names_and_types,
  std::vector<rosgraph_msgs::msg::Service> & service_endpoints,
  std::vector<rosgraph_msgs::msg::Topic> & topic_endpoints)
{
  std::map<std::string, const rosgraph_msgs::msg::Service *> services_by_name;
  for (const auto & s : service_endpoints) {
    services_by_name[s.name] = &s;
  }
  std::map<std::string, const rosgraph_msgs::msg::Topic *> topics_by_name;
  for (const auto & t : topic_endpoints) {
    topics_by_name[t.name] = &t;
  }

  std::vector<rosgraph_msgs::msg::Action> actions;
  std::set<std::string> consumed_services;
  std::set<std::string> consumed_topics;

  for (const auto & [action_name, types] : action_names_and_types) {
    rosgraph_msgs::msg::Action action;
    action.name = action_name;
    const std::string type_name = first_type(types);
    const std::string base = action_name + kActionInfix;

    for (const auto & suffix : kActionServiceSuffixes) {
      const std::string const_name = base + suffix;
      auto it = services_by_name.find(const_name);
      rosgraph_msgs::msg::Service service;
      if (it != services_by_name.end()) {
        service = *it->second;
        consumed_services.insert(const_name);
      } else {
        // Constituent absent from the graph for this node; build a placeholder
        // so the Action message stays well-formed.
        service = build_service(const_name, type_name, type_name);
      }
      if (suffix == "send_goal") {
        action.send_goal = service;
      } else if (suffix == "get_result") {
        action.get_result = service;
      } else {  // cancel_goal
        action.cancel_goal = service;
      }
    }

    for (const auto & suffix : kActionTopicSuffixes) {
      const std::string const_name = base + suffix;
      auto it = topics_by_name.find(const_name);
      rosgraph_msgs::msg::Topic topic;
      if (it != topics_by_name.end()) {
        topic = *it->second;
        consumed_topics.insert(const_name);
      } else {
        topic.name = const_name;
        topic.type = interface_type(type_name);
      }
      if (suffix == "feedback") {
        action.feedback = topic;
      } else {  // status
        action.status = topic;
      }
    }

    actions.push_back(std::move(action));
  }

  // Remove folded constituents from the flat lists, in place.
  service_endpoints.erase(
    std::remove_if(
      service_endpoints.begin(),
      service_endpoints.end(),
      [&consumed_services](const rosgraph_msgs::msg::Service & s) { return consumed_services.count(s.name) > 0; }),
    service_endpoints.end());
  topic_endpoints.erase(
    std::remove_if(
      topic_endpoints.begin(),
      topic_endpoints.end(),
      [&consumed_topics](const rosgraph_msgs::msg::Topic & t) { return consumed_topics.count(t.name) > 0; }),
    topic_endpoints.end());

  sort_actions(actions);
  return actions;
}

}  // namespace nodl_observe
