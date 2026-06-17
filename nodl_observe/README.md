# nodl_observe

Observe a **running** node and produce its runtime description as a
`rosgraph_msgs/Node` message — stage one of the Observe → Describe pipeline:

```
running node --[ Observe ]--> rosgraph_msgs/Node --[ Describe ]--> NoDL document
```

Observe *records*: everything observable about the node — every endpoint
(including infrastructure like `/rosout`, `/parameter_events`, and the
parameter services), actual QoS, type hashes, parameter descriptors and
current values — unfiltered. Deciding what counts as "the node's interface"
is *interpretation*, which belongs to Describe.

This is the **C++ (`ament_cmake`) reimplementation** of the observer. It
exposes a reusable `observe_node(...)` library plus a thin `observe`
executable.

## API

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nodl_observe/observe.hpp"

rclcpp::init(argc, argv);
auto node = std::make_shared<rclcpp::Node>("observer");

nodl_observe::Options opts;          // timeout{5.0s}, include_parameters{true}
auto msg = nodl_observe::observe_node(*node, "/my_namespace/my_node", opts);
```

`observe_node` never creates or spins its own node; it uses the caller's node
for all graph queries and (unless `opts.include_parameters == false`) the
target's parameter services. `opts.timeout` is a ceiling shared across
discovery/stability polling and all parameter round-trips — it returns as soon
as the graph settles. **The caller must not be spinning `node` on another
thread concurrently**: parameter collection drives async futures via a
short-lived internal executor that owns the node for that window.

`nodl_observe::latched_qos()` exposes the QoS profile of the latched CLI
publish (`reliable + transient_local + keep_last(1)`).

## The `observe` executable

```
observe <node_fqn> [--timeout SECONDS] [--no-parameters] [--spin-seconds N] [--topic TOPIC]
```

Defaults: `--timeout 5.0`, parameters on, `--spin-seconds 0` (spin forever
until SIGINT), `--topic /nodl/observed_node`. It observes the target, then
**latch-publishes** the resulting `rosgraph_msgs/Node` on `--topic`
(transient_local) and stays alive so late subscribers can still fetch the
sample. Exit code `1` if the target node never appears within the timeout.

The serialized `Node` is the language boundary for the future `ros2 nodl
describe` verb, which is a thin Python wrapper that **shells out** to this
`observe` binary and renders the result (verb wiring is a **follow-up**).

## Observability limits

Not every `Node.msg` field is observable from an external process:

| Entity | What is filled |
|---|---|
| publishers / subscriptions | name, type, QoS, and RIHS type hash via the info-by-topic graph queries (`get_{publishers,subscriptions}_info_by_topic`) |
| service servers / clients | name and types only; **QoS is reported as `*_UNKNOWN`** — there is no info-by-service API in rclcpp/rmw — and the type hash is unset (message default: version 1, all-zero value) |
| action servers / clients | derived: the hidden `<action>/_action/*` entities are folded into each `Action` entry (topics get real QoS, services get UNKNOWN). Orphan `_action/*` entities stay flat — nothing is discarded |

Action graph queries drop to the `rcl_action` C API
(`rcl_action_get_{server,client}_names_and_types_by_node`); there is no
`rclcpp_action` wrapper for them.

**Per-RMW gaps surface honestly rather than being papered over.** Reliability,
durability, and deadline come through everywhere; the known gaps (e.g.
`rmw_fastrtps_cpp` on jazzy dropping history/depth over discovery,
`rmw_cyclonedds_cpp` reporting a `KEEP_ALL` queue's depth as `0`) are recorded
faithfully, never fabricated.

Infinite/unspecified QoS durations (and any value overflowing
`builtin_interfaces/Duration.sec`) are canonicalised to a fixed, CDR-valid
sentinel of `{sec = INT32_MAX, nanosec = 0}`, applied uniformly on every distro
(this differs from graph-monitor's `{0, 0}`).

Requires a `rosgraph_msgs` that provides `Node.msg`. **Humble (pre-Iron) is
supported as a runtime target**, message-identical to Iron+: the REP-2011 topic
type hash and the `BEST_AVAILABLE` QoS enum do not exist there, so on Humble the
topic type hash is left unset (same honest-unknown state as services) and
`BEST_AVAILABLE` is compiled out — the differences live only in those unfilled
fields, never in the message shape. This is gated by the `ROS2_${ROS_DISTRO}`
compile definition, and Humble is built + tested in CI with its own fixtures.

## Tests

- **Unit tests (gtest)** cover the pure builders with no executor/graph:
  `test_qos.cpp` (QoS enum mapping incl. `BEST_AVAILABLE`, durations carried,
  infinite-duration clamp, all-unknown), `test_endpoints.cpp` (topic type
  hash + QoS carried, name/type-only fallback, sorting, service UNKNOWN QoS +
  default hash), `test_actions.cpp` (fold + remove-from-flat, orphan stays
  flat, placeholders, sorting), `test_parameters.cpp` (parameter pairing /
  sorting / length mismatch, `split_fqn`), and `test_collect_parameters.cpp`
  (the graceful-degradation path: an absent/unresponsive target yields empty
  arrays, never throws — this one needs a live rclcpp context).
- **Integration test (pytest)**, `test/test_observe_integration.py` (authored
  separately), spins scenario nodes, runs the `observe` binary, and compares
  the observed `Node` field-by-field against **MCAP fixtures** (which replace
  the previous YAML goldens; regenerated behind a flag).
