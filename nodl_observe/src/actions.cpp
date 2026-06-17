// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
#include "nodl_observe/actions.hpp"

#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rcl/allocator.h"
#include "rcl/graph.h"
#include "rcl_action/graph.h"
#include "rcl_action/names.h"

namespace nodl_observe
{

namespace
{

using NamesAndTypes = std::map<std::string, std::vector<std::string>>;

// Convert a populated rcl_names_and_types_t into the std::map shape used by the
// pure builders.  Does not take ownership; the caller still fini's the struct.
NamesAndTypes to_map(const rcl_names_and_types_t & nat)
{
  NamesAndTypes out;
  for (size_t i = 0; i < nat.names.size; ++i) {
    std::vector<std::string> types;
    const rcutils_string_array_t & type_array = nat.types[i];
    types.reserve(type_array.size);
    for (size_t j = 0; j < type_array.size; ++j) {
      types.emplace_back(type_array.data[j]);
    }
    out.emplace(std::string(nat.names.data[i]), std::move(types));
  }
  return out;
}

// Shared body for the server / client variants, parameterised on the rcl_action
// query function pointer.
NamesAndTypes query_action_names_and_types(
  rclcpp::Node & node,
  const std::string & name,
  const std::string & ns,
  rcl_ret_t (*query_fn)(const rcl_node_t *, rcl_allocator_t *, const char *, const char *, rcl_names_and_types_t *),
  const char * what)
{
  rcl_node_t * node_handle = node.get_node_base_interface()->get_rcl_node_handle();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_names_and_types_t nat = rcl_get_zero_initialized_names_and_types();

  const rcl_ret_t ret = query_fn(node_handle, &allocator, name.c_str(), ns.c_str(), &nat);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error(
      std::string("Failed to query action ") + what + " names and types for node '" + ns + "/" + name + "'");
  }

  NamesAndTypes result = to_map(nat);

  const rcl_ret_t fini_ret = rcl_names_and_types_fini(&nat);
  if (fini_ret != RCL_RET_OK) {
    throw std::runtime_error(std::string("Failed to finalise action ") + what + " names and types");
  }

  return result;
}

}  // namespace

NamesAndTypes get_action_server_names_and_types_by_node(
  rclcpp::Node & node, const std::string & name, const std::string & ns)
{
  return query_action_names_and_types(node, name, ns, rcl_action_get_server_names_and_types_by_node, "server");
}

NamesAndTypes get_action_client_names_and_types_by_node(
  rclcpp::Node & node, const std::string & name, const std::string & ns)
{
  return query_action_names_and_types(node, name, ns, rcl_action_get_client_names_and_types_by_node, "client");
}

}  // namespace nodl_observe
