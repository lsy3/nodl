# nodl_observe architecture

`nodl_observe` turns a **running** ROS 2 node into a `rosgraph_msgs/Node` (the
input "Describe" #53 converts to a NoDL document). It is layered by one question —
**does it touch the live ROS graph?** — so all graph I/O lives in one orchestrator
(`observe.cpp`), the pure builders are unit-testable with no ROS, and node
ownership is pushed to the edges (the CLI binary owns a node; graph-monitor reuses
the library in-process).

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
