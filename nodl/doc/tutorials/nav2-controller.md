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
topic, service, and action endpoints, but it cannot yet compose reusable fragments, model lifecycle state or bond
ownership, express TF frame semantics, generate bindings, or conform a running node.
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

### 3. Compose framework capabilities with node-owned endpoints

```bash
ros2 nodl compose nodl/controller_server.observed.nodl.yaml \
  --base nav2:lifecycle-node \
  --fragment nav2:bond \
  --fragment nav2:tf-consumer \
  --fragment nav2:controller-qos \
  --output nodl/controller_server.nodl.yaml
ros2 nodl validate nodl/controller_server.nodl.yaml
```

Composition has explicit ownership boundaries:

| Capability | Owner | What NoDL describes |
|---|---|---|
| Lifecycle services and state transitions | `nav2::LifecycleNode` base | Shared lifecycle endpoint contract; not controller behavior |
| Bond | Nav2 base during activation | Shared bond interface and activation ownership |
| TF transport | Nav2 base/costmap integration | `/tf` and `/tf_static` topic participation |
| Frame requirements | System configuration | Required transform relationships, verified separately from topic endpoints |
| `follow_path` and controller topics | `ControllerServer` | Action and topic interface specific to this node |

The last distinction matters: `/tf` and `/tf_static` do not, by themselves, prove that a configured `map → odom →
base_link` transform chain exists.

### 4. Generate the interface binding without generating control behavior

```bash
ros2 nodl generate nodl/controller_server.nodl.yaml \
  --language cpp --output generated/controller_server
colcon build --packages-select nav2_controller
```

Generation binds declared ROS interfaces to the existing C++ implementation. It does not generate controller plugins,
local-costmap behavior, TF lookup logic, control-loop timing, real-time scheduling, or recovery behavior.

### 5. Conform a running lifecycle node

Bring the node through its configured lifecycle transition, then verify its interface:

```bash
ros2 lifecycle set /controller_server configure
ros2 lifecycle set /controller_server activate
ros2 nodl conform /controller_server --file nodl/controller_server.nodl.yaml
```

Conformance compares expected endpoints and QoS with observation. Separate system checks verify lifecycle state,
required TF transforms, and a successful `FollowPath` request; they are not inferred merely from endpoint presence.

### 6. Demonstrate a composition failure

Launch an intentionally incomplete composed variant that omits the Nav2 bond capability, then run conformance:

```bash
ros2 nodl conform /controller_server --file nodl/controller_server.nodl.yaml
```

The semantic diff attributes the missing bond interface to the `nav2:bond` fragment rather than incorrectly blaming
`ControllerServer`'s application endpoints. Restore the fragment, reconform, and confirm the node is active.

## What works today

On current `main`, a user can observe `/controller_server` as raw graph data and validate a manually authored document
containing direct endpoints. The current schema deliberately rejects `base` and `fragments`, and current observation
does not recover lifecycle state, bond ownership, or semantic TF frame requirements.

This tutorial therefore defines the acceptance criteria for future work:

- reusable, versioned Nav2 lifecycle and bond documents;
- fragment provenance and ownership-aware semantic diffs;
- observation and conformance for actions and QoS;
- explicit separation of TF transport endpoints from frame-relationship checks; and
- C++ binding generation that leaves Nav2 control behavior untouched.

## Why ControllerServer is the composition example

`ControllerServer` has a compact node-specific surface but inherits meaningful framework behavior. That makes it a
useful test of whether NoDL composition adds real information: the result must identify what comes from Nav2, what
comes from the controller, and what remains a system-level operational requirement.
