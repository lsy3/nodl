// SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Layer-1 unit tests for the pure parameter pairing builder and split_fqn.
// Ports test_observe.py::TestParameterPairing and ::TestSplitFqn.  The async
// collect_parameters degradation path needs a live node/executor, so it is left
// to the integration test rather than re-faked here.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "nodl_observe/observe.hpp"
#include "nodl_observe/parameters.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

using nodl_observe::build_parameters;
using nodl_observe::split_fqn;

namespace
{

rcl_interfaces::msg::ParameterDescriptor descriptor(const std::string & name, bool read_only = false)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.name = name;
  d.read_only = read_only;
  return d;
}

rcl_interfaces::msg::ParameterValue int_value(int64_t v)
{
  rcl_interfaces::msg::ParameterValue value;
  value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  value.integer_value = v;
  return value;
}

}  // namespace

TEST(ParameterPairing, PairedAndSortedByName)
{
  std::vector<std::string> names{"z_param", "a_param"};
  std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors{descriptor("z_param"), descriptor("a_param", true)};
  std::vector<rcl_interfaces::msg::ParameterValue> values{int_value(1), int_value(2)};

  const auto [out_d, out_v] = build_parameters(names, descriptors, values);
  ASSERT_EQ(out_d.size(), 2u);
  EXPECT_EQ(out_d[0].name, "a_param");
  EXPECT_EQ(out_d[1].name, "z_param");
  // Values stay matched 1:1 with their descriptor after sorting.
  EXPECT_EQ(out_v[0].integer_value, 2);  // a_param
  EXPECT_EQ(out_v[1].integer_value, 1);  // z_param
  EXPECT_TRUE(out_d[0].read_only);
}

TEST(ParameterPairing, LengthMismatchKeepsOnlyCompletePairs)
{
  std::vector<std::string> names{"a", "b"};
  std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors{descriptor("a")};
  std::vector<rcl_interfaces::msg::ParameterValue> values{int_value(1), int_value(2)};

  const auto [out_d, out_v] = build_parameters(names, descriptors, values);
  ASSERT_EQ(out_d.size(), 1u);
  EXPECT_EQ(out_d[0].name, "a");
  EXPECT_EQ(out_v.size(), 1u);
}

TEST(ParameterPairing, EmptyInputs)
{
  const auto [out_d, out_v] = build_parameters({}, {}, {});
  EXPECT_TRUE(out_d.empty());
  EXPECT_TRUE(out_v.empty());
}

TEST(SplitFqn, Cases)
{
  EXPECT_EQ(split_fqn("/talker"), (std::pair<std::string, std::string>{"talker", "/"}));
  EXPECT_EQ(split_fqn("/ns/talker"), (std::pair<std::string, std::string>{"talker", "/ns"}));
  EXPECT_EQ(split_fqn("/ns/sub/talker"), (std::pair<std::string, std::string>{"talker", "/ns/sub"}));
  EXPECT_EQ(split_fqn("/ns/talker/"), (std::pair<std::string, std::string>{"talker", "/ns"}));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
