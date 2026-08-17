# Nav2 ControllerServer: reusable lifecycle composition

This tutorial uses Nav2's C++ `nav2_controller::ControllerServer` to show why NoDL needs reusable node-base documents
and ownership-aware composition. It is a framework composition example, not a replacement for the Dummy robot
migration tutorial.

The reviewed source is pinned to Navigation2 commit
[`b356ac1`](https://github.com/ros-navigation/navigation2/blob/b356ac1f8512bcc5f9b595562139f5b02095319b/nav2_controller/src/controller_server.cpp).
`ControllerServer` derives from `nav2::LifecycleNode`; when configured, it creates the `follow_path` action server,
`transformed_global_plan` and `tracking_feedback` publishers, and a speed-limit subscription. On activation it creates
a bond. Its local costmap and plugins also depend on TF.

:::{warning}
**Not yet implemented.** This walkthrough is the target NoDL product experience. Current NoDL can validate direct
topic, service, and action endpoints, resolve schema-level `include` references, preserve the include tree, and carry
opaque `codegen` metadata. The Nav2 capability documents, tools that interpret their metadata and provenance,
lifecycle-state and bond-ownership modeling, TF frame semantics, generated bindings, and running-node conformance do
not exist yet. The current ament helper registers executable documents only; this tutorial also assumes a future
`ament_nodl_register_document()` helper for reusable capability documents.
:::

## Tutorial

### 1. Run a configured Nav2 system

Start a Nav2 bringup with a robot, map, and parameter file suitable for the target environment:

```bash
ros2 launch nav2_bringup navigation_launch.py \
  map:=/path/to/map.yaml params_file:=/path/to/nav2_params.yaml
```

The target node is `/controller_server`. Its resolved topic names, QoS, and some enabled interfaces depend on the
Nav2 parameter file and namespace; the tutorial does not treat a source-level default as a runtime guarantee.

### 2. Describe the application-specific interface

```bash
ros2 nodl describe /controller_server \
  --output nodl/controller_server.observed.nodl.yaml
```

The draft records what the configured running node exposes. At minimum, the source-level application interface
includes the `follow_path` `nav2_msgs/action/FollowPath` action server, `transformed_global_plan`
`nav_msgs/msg/Path` publisher, `tracking_feedback` `nav2_msgs/msg/TrackingFeedback` publisher, and a
`nav2_msgs/msg/SpeedLimit` subscription. The resolved document must retain the actual names and QoS from observation.

### 3. Publish reusable Nav2 capability documents

`nav2_common` publishes one document for each framework capability. For example, the lifecycle document contains the
stable lifecycle service interface and no controller-specific endpoint:

```yaml
# nav2_common/nodl/lifecycle_node.nodl.yaml
nodl_version: 2
codegen:
  cpp:
    role: base_class
    header: nav2_ros_common/lifecycle_node.hpp
    class: nav2::LifecycleNode
service_servers:
  - {name: change_state, type: lifecycle_msgs/srv/ChangeState}
  - {name: get_state, type: lifecycle_msgs/srv/GetState}
  - {name: get_available_states, type: lifecycle_msgs/srv/GetAvailableStates}
  - {name: get_available_transitions, type: lifecycle_msgs/srv/GetAvailableTransitions}
  - {name: get_transition_graph, type: lifecycle_msgs/srv/GetAvailableTransitions}
```

The capability package registers this document by a stable package/document identity:

```cmake
ament_nodl_register_document(lifecycle_node
  FILE nodl/lifecycle_node.nodl.yaml
  PACKAGE nav2_common)
```

The same package publishes `bond.nodl.yaml`, `tf_consumer.nodl.yaml`, and `controller_qos.nodl.yaml`. Each document
contains endpoints that belong to that capability. A future `ament_nodl_register_document()` stores them under the
same `nodl_nodes` ament resource type that `include` already resolves, using the key `nav2_common__lifecycle_node`.
The include tree keeps the lifecycle document and its `codegen.cpp` metadata separate from the root controller
document, so a generator can identify the base class that contributed the lifecycle endpoints.

### 4. Include framework capabilities with node-owned endpoints

The authored document includes reusable Nav2 documents and declares only the controller-specific endpoints. The
`nav_msgs/msg/Path` publisher is a compact, concrete example of the value: it remains visible at the node level while
the lifecycle and TF transport contract stays reusable.

```yaml
# nodl/controller_server.nodl.yaml
nodl_version: 2
include:
  - ref: nodl://nav2_common/lifecycle_node
  - ref: nodl://nav2_common/bond
  - ref: nodl://nav2_common/tf_consumer
  - ref: nodl://nav2_common/controller_qos
publishers:
  - name: transformed_global_plan
    type: nav_msgs/msg/Path
    qos: {history: SYSTEM_DEFAULT, reliability: SYSTEM_DEFAULT}
  - name: tracking_feedback
    type: nav2_msgs/msg/TrackingFeedback
    qos: {history: SYSTEM_DEFAULT, reliability: SYSTEM_DEFAULT}
subscriptions:
  - name: speed_limit
    type: nav2_msgs/msg/SpeedLimit
    qos: {history: SYSTEM_DEFAULT, reliability: SYSTEM_DEFAULT}
action_servers:
  - name: follow_path
    type: nav2_msgs/action/FollowPath
```

```bash
ros2 nodl validate nodl/controller_server.nodl.yaml
```

Includes merge endpoint documents. The composed contract also has explicit ownership boundaries:

| Capability | Owner | What NoDL describes |
|---|---|---|
| Lifecycle services and state transitions | `nav2::LifecycleNode` base | Shared lifecycle endpoint contract; not controller behavior |
| Bond | Nav2 base during activation | Shared bond interface and activation ownership |
| TF transport | Nav2 base/costmap integration | `/tf` and `/tf_static` topic participation |
| Frame requirements | System configuration | Required transform relationships, verified separately from topic endpoints |
| `follow_path` and controller topics | `ControllerServer` | Action and topic interface specific to this node |

The last distinction matters: `/tf` and `/tf_static` do not, by themselves, prove that a configured `map → odom →
base_link` transform chain exists.

### 5. Generate the interface binding without generating control behavior

```bash
ros2 nodl generate nodl/controller_server.nodl.yaml \
  --language cpp --output generated/controller_server
colcon build --packages-select nav2_controller
```

Generation resolves the document with its include tree, interprets the included `codegen.cpp` metadata, and binds the
declared ROS interfaces to the existing C++ implementation. It does not generate controller plugins, local-costmap
behavior, TF lookup logic, control-loop timing, real-time scheduling, or recovery behavior.

### 6. Conform a running lifecycle node

Bring the node through its configured lifecycle transition, then verify its interface:

```bash
ros2 lifecycle set /controller_server configure
ros2 lifecycle set /controller_server activate
ros2 nodl conform /controller_server --file nodl/controller_server.nodl.yaml
```

Conformance compares expected endpoints and QoS with observation. Separate system checks verify lifecycle state,
required TF transforms, and a successful `FollowPath` request; they are not inferred merely from endpoint presence.

### 7. Demonstrate an include failure

Remove `nodl://nav2_common/bond` from the document, launch the intentionally incomplete variant, then run
conformance:

```bash
ros2 nodl conform /controller_server --file nodl/controller_server.nodl.yaml
```

The semantic diff attributes the missing bond interface to the missing include rather than incorrectly blaming
`ControllerServer`'s application endpoints. Restore the include, reconform, and confirm the node is active.

## What works today

On current `main`, a user can observe `/controller_server` as raw graph data and validate a manually authored document
containing direct endpoints. `nodl_schema` can resolve registered `include` references, but the required Nav2
capability documents and their package-level registration helper are not implemented yet. `load_nodl_with_doc_tree()`
exposes include provenance and `codegen` metadata, but no generator, semantic diff, or conformance tool consumes them.
Current observation also does not recover lifecycle state, bond ownership, or semantic TF frame requirements.

This tutorial therefore defines the acceptance criteria for future work:

- reusable, versioned Nav2 lifecycle and bond documents;
- package-level registration of reusable documents;
- generator, semantic-diff, and conformance use of the existing include tree;
- observation and conformance for actions and QoS;
- explicit separation of TF transport endpoints from frame-relationship checks; and
- C++ binding generation that leaves Nav2 control behavior untouched.

## Why ControllerServer is the composition example

`ControllerServer` has a compact node-specific surface but inherits meaningful framework behavior. That makes it a
useful test of whether NoDL composition adds real information: the result must identify what comes from Nav2, what
comes from the controller, and what remains a system-level operational requirement.
