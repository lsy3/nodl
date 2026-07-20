# nodl_observe architecture

`nodl_observe` turns a **running** ROS 2 node into a `rosgraph_msgs/Node`
message — the input that "Describe" (#53) converts into a NoDL document. The
design is layered by a single question: **does it touch the live ROS graph?**
Pure data-shaping sits at the bottom (unit-testable with no ROS), all graph I/O
is isolated in one orchestrator, and node ownership is pushed to the edges.

## Data flow

```
                          LIVE ROS GRAPH  (RMW / middleware discovery)
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
      ▼  (separate process, via the middleware)
  ros2nodl/verb/describe.py   (Python)
  spawn binary → subscribe latched → rosidl_runtime_py → YAML/JSON
```

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
