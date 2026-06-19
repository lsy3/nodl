<!-- SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# nodl_observe architecture

`nodl_observe` turns a **running** ROS 2 node into a `rosgraph_msgs/Node`
message — the input that "Describe" (#53) converts into a NoDL document. The
design is layered by a single question: **does it touch the live ROS graph?**
Pure data-shaping sits at the bottom (unit-testable with no ROS), all graph I/O
is isolated in one orchestrator, and node ownership is pushed to the edges.

## Data flow

```
                          LIVE ROS GRAPH  (DDS discovery)
                                  │
   ┌──────────────────────────────┼──────────────────────────────┐
   │ NodeGraphInterface           │ rcl_action C API   AsyncParametersClient
   │ (pub/sub/srv by node,        │ (action graph)     (~/list,describe,get_
   │  *_info_by_topic)            │                     parameters)
   └──────────────────────────────┼──────────────────────────────┘
                                  ▼
        ┌─────────────────────────────────────────────────┐
        │  observe.cpp :: observe_node(node, fqn, opts)    │   ◄── the ONLY
        │  wait_for_node → wait_for_stable_graph →         │       graph-driving
        │  collect_endpoints → fold actions → parameters   │       layer
        └─────────────────────────────────────────────────┘
                 │ hands raw names/types, TopicEndpointInfo,
                 │ rcl_action names, parameter responses to ↓
                 ▼
   ┌───────────────────────────────────────────────────────────────┐
   │  PURE BUILDERS  (no graph access — unit-tested in isolation)    │
   │  qos.cpp        QoS enums + durations → QoSProfile msg          │
   │  endpoints.cpp  → Topic / Service / Action sub-msgs; fold;      │
   │                   type hashes; sort                             │
   │  parameters.cpp build_parameters() pairs descriptors+values     │
   └───────────────────────────────────────────────────────────────┘
                 │
                 ▼
        rosgraph_msgs/Node   (sorted, deterministic)
                 │
      ┌──────────┴───────────────────────────────┐
      ▼                                           ▼
  observe_main.cpp                        graph-monitor
  (the `observe` binary)                  links libnodl_observe directly,
  init → observe_node → latch-publish     reuses the pure builders — no
  on /nodl/observed_node → spin           binary, no verb
      │
      ▼  (separate process, DDS)
  ros2nodl/verb/describe.py   (Python)
  spawn binary → subscribe latched → rosidl_runtime_py → YAML/JSON
```

## Modules

| File | Role | Touches graph? |
|---|---|---|
| `qos.{hpp,cpp}` | QoS enum→msg mapping (explicit `switch`), the `{INT32_MAX,0}` duration clamp, `unknown_qos_msg`, `latched_qos` | No — **pure** |
| `endpoints.{hpp,cpp}` | Build `Topic`/`Service`/`Action` sub-msgs from raw names+types & `TopicEndpointInfo`; `fold_actions`; type hashes; sorting | No — **pure** |
| `actions.{hpp,cpp}` | Wrap the `rcl_action` C API to get action names/types by node | Yes (C API) |
| `parameters.{hpp,cpp}` | `build_parameters` (pure pairing) + `collect_parameters` (drives `AsyncParametersClient`, graceful degradation) | Mixed |
| `observe.{hpp,cpp}` | `observe_node` orchestrator + `split_fqn` | **Yes — the hub** |
| `observe_main.cpp` | The `observe` executable: arg parsing, node lifecycle, latched publish | owns the node |
| `../ros2nodl/.../verb/describe.py` | Python verb — shells out to the binary, subscribes, renders | separate process |

## `observe_node` step by step

1. **`split_fqn`** — `/ns/talker` → `("talker", "/ns")`.
2. **`wait_for_node`** — poll `get_node_names_and_namespaces()` until the target
   appears, else raise `NodeNotFoundError`.
3. **`wait_for_stable_graph`** — there is no "discovery complete" signal, so poll
   the four by-node queries (publisher / subscriber / service-server /
   service-client names+types) until the set is **unchanged for 3 consecutive
   200 ms samples**.
4. **`collect_endpoints`** — per topic, `get_{publishers,subscriptions}_info_by_topic`
   → filter to this node → `build_topic` (QoS + type hash); `build_service`
   (always `*_UNKNOWN`); query `rcl_action` → `fold_actions` moves the hidden
   `<action>/_action/{send_goal,get_result,cancel_goal,feedback,status}` entities
   out of the flat lists into each `Action` (orphans with no parent stay flat —
   nothing is discarded).
5. **parameters** (unless `include_parameters == false`) — a short-lived
   `SingleThreadedExecutor` drives an `AsyncParametersClient`
   (`list` → `describe` + `get`); any failure degrades to empty arrays.
6. Assemble the `Node`, every array **sorted** for deterministic output.

## Why this shape

- **Pure builders = cheap correctness.** All the tricky logic (QoS mapping,
  action folding, parameter pairing) takes plain data in and returns messages, so
  the gtests assert exact outputs with no ROS spun up.
- **One graph-driving layer.** Only `observe.cpp` polls discovery, so the
  "wait for the graph to settle" race lives in exactly one place.
- **No owned node in the core.** `observe_node` borrows the caller's node, so
  graph-monitor reuses the library in-process while the CLI gets node ownership +
  latched publish through the thin `observe` binary. The serialized `Node` is the
  single boundary to the Python verb — no pybind, no C++↔Python conversion.

## Test layers

- **Unit (gtest)** — the pure builders, with no executor/graph (`test_qos.cpp`,
  `test_endpoints.cpp`, `test_actions.cpp`, `test_parameters.cpp`), plus
  `test_collect_parameters.cpp` for the degradation path (needs a live context).
- **Integration (pytest)** — `test_observe_integration.py` spins scenario graphs,
  runs the `observe` binary, and compares the observed `Node` field-by-field
  against the MCAP fixtures in `test/fixtures/` (see `test/fixtures/README.md`).
  `test/mcap_fixtures.py` is the human-readable `print`/`diff` helper.
