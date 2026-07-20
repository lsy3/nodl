# nodl_observe

Observe a **running** node and produce its runtime description as a
`rosgraph_msgs/Node` message — stage one of the Observe → Describe pipeline:

```
running node --[ Observe ]--> rosgraph_msgs/Node --[ Describe ]--> NoDL document
```

Observe *records* everything observable — every endpoint (including infrastructure
like `/rosout`, `/parameter_events`, the parameter services), actual QoS, type
hashes, and parameters — unfiltered. Deciding what counts as "the node's interface"
is *interpretation*, and belongs to Describe.

This is the **C++ (`ament_cmake`) reimplementation**: a reusable `observe_node(...)`
library plus a thin `observe` executable.

## API

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nodl_observe/observe.hpp"

rclcpp::init(argc, argv);
auto node = std::make_shared<rclcpp::Node>("observer");

nodl_observe::Options opts;          // timeout{5.0s}, include_parameters{true}
auto msg = nodl_observe::observe_node(*node, "/my_namespace/my_node", opts);
```

`observe_node` never creates or spins its own node; it uses the caller's node for
graph queries and (unless `include_parameters == false`) the target's parameter
services. `timeout` is a ceiling across discovery, stability polling, and parameter
round-trips. **The caller must not spin `node` concurrently** — parameter collection
drives async futures via a short-lived internal executor that owns the node.
`nodl_observe::latched_qos()` is the latched-publish profile (`reliable +
transient_local + keep_last(1)`).

## The `observe` executable

```
observe <node_fqn> [--timeout SECONDS] [--no-parameters] [--spin-seconds N] [--topic TOPIC]
```

Defaults: `--timeout 5.0`, parameters on, `--spin-seconds 0` (spin until SIGINT),
`--topic /nodl/observed_node`. It observes the target, **latch-publishes** the
`rosgraph_msgs/Node` on `--topic`, and stays alive for late subscribers. Exit `1` if
the node never appears within the timeout. The serialized `Node` is the language
boundary for the future `ros2 nodl describe` verb (a thin Python wrapper that shells
out to this binary).

## Observability limits

Not every `Node.msg` field is observable externally:

| Entity | What is filled |
|---|---|
| publishers / subscriptions | name, type, QoS, RIHS type hash (`get_{publishers,subscriptions}_info_by_topic`) |
| service servers / clients | name + types only; **QoS is `*_UNKNOWN`** (no info-by-service API) and the type hash is unset |
| action servers / clients | the hidden `<action>/_action/*` entities are folded into each `Action` (topics keep real QoS, services UNKNOWN); orphans stay flat |

Action graph queries use the `rcl_action` C API (no `rclcpp_action` wrapper). Per-RMW
gaps are recorded faithfully, never fabricated (e.g. jazzy's `rmw_fastrtps_cpp` drops
history/depth over discovery; `rmw_cyclonedds_cpp` reports a `KEEP_ALL` depth as 0).
Infinite/overflowing QoS durations are canonicalised to `{sec = INT32_MAX, nanosec =
0}` on every distro.

**Humble (pre-Iron) is supported**, message-identical to Iron+: the REP-2011 type
hash and `BEST_AVAILABLE` QoS enum don't exist there, so on Humble the topic hash is
left unset and `BEST_AVAILABLE` is compiled out (gated by the `ROS2_${ROS_DISTRO}`
define). Requires a `rosgraph_msgs` that provides `Node.msg`.
