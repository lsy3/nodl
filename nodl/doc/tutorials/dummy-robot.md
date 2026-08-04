# Dummy robot: from observation to a visible system

The `dummy_robot` demo combines a map server, fake joint states, fake laser data, `robot_state_publisher`, TF, launch,
and RViz. It is the preferred ROSCon hero because the robot and laser result remain visible after the interface work.

The tutorial starts and finishes with the complete robot. NoDL operations remain node-scoped, so the detailed
walkthrough later selects `dummy_laser` as one worked node without treating it as the whole demo.

The reviewed source is
[`dummy_sensors/src/dummy_laser.cpp`](https://github.com/ros2/demos/blob/f8d20abaec2be76c7062b2a0242ae6f82e2857b8/dummy_robot/dummy_sensors/src/dummy_laser.cpp).

:::{note}
This is a **prototype tutorial**. It exercises the real robot and the current observation/validation tools. Generated
NoDL drafts, composition, forward generation, semantic diff, and conformance remain design targets.
:::

## 1. Start the existing robot

Build the upstream packages:

```bash
colcon build --packages-select dummy_map_server dummy_sensors dummy_robot_bringup
```

Source the workspace, then start the demo:

```bash
ros2 launch dummy_robot_bringup dummy_robot_bringup_launch.py
```

The launch starts:

- `/dummy_map_server`;
- `/robot_state_publisher`;
- `/dummy_joint_states`;
- `/dummy_laser`;
- RViz.

RViz should display the robot and its changing laser scan.

List the nodes that make up the running system:

```bash
ros2 node list
```

The expected application nodes are:

```text
/dummy_joint_states
/dummy_laser
/dummy_map_server
/robot_state_publisher
```

`rviz2` also appears when the visualizer is running.

## 2. Observe the system one node at a time

NoDL currently describes one node at a time. There is no `describe-system` command, so record each participating node
explicitly:

```bash
mkdir -p /tmp/dummy_robot_observed
ros2 nodl describe /dummy_map_server --no-params \
  -o /tmp/dummy_robot_observed/dummy_map_server.yaml
ros2 nodl describe /dummy_joint_states --no-params \
  -o /tmp/dummy_robot_observed/dummy_joint_states.yaml
ros2 nodl describe /dummy_laser --no-params \
  -o /tmp/dummy_robot_observed/dummy_laser.yaml
ros2 nodl describe /robot_state_publisher --no-params \
  -o /tmp/dummy_robot_observed/robot_state_publisher.yaml
```

This explicit list is useful documentation: it shows the system boundary and preserves each node's ownership. A future
launch- or system-level manifest can reference these node documents without changing their node-scoped meaning.

:::{warning}
The saved files are raw `rosgraph_msgs/Node` serializations. Current `main` does not convert them into NoDL documents.
:::

This step exposes the first missing CLI behavior. The desired default is for `ros2 nodl describe NODE` to emit a
schema-valid draft NoDL document. Raw observation should remain available through an explicit diagnostic option.

## 3. Curate one worked node: `dummy_laser`

The source creates a `sensor_msgs/msg/LaserScan` publisher on relative topic `scan` with depth 10. Its loop computes
range values, timestamps messages, and sets `single_rrbot_hokuyo_link` as the message frame.

Inspect the `dummy_laser` observation from the system inventory, then curate this source-level document:

```{literalinclude} ../../../examples/nodl_tutorial_verification/nodl/dummy_laser.nodl.yaml
:language: yaml
```

Validate it:

```bash
ros2 nodl validate examples/nodl_tutorial_verification/nodl/dummy_laser.nodl.yaml
```

The document describes the publisher. It does not describe the 30 Hz loop, generated range values, message
timestamps, or real-time behavior.

The manual translation exposes another missing part of Describe: it needs a policy for framework endpoints such as
`/rosout`, parameter services, and `/parameter_events`. The default draft should focus on the node's public application
interface, while an option should retain infrastructure for diagnostics.

## 4. Verify behavior outside NoDL

Confirm that laser messages still arrive:

```bash
ros2 topic echo /scan --once
```

Confirm the laser frame participates in the running TF tree:

```bash
ros2 run tf2_ros tf2_echo world single_rrbot_hokuyo_link
```

Stop `tf2_echo` after it reports a transform.

### TF topics are not frame semantics

`robot_state_publisher` publishes transforms used to place the laser scan in RViz. A NoDL document can describe its
`/tf` and `/tf_static` topic endpoints. Those endpoints do not prove that a particular frame path exists.

For this robot, `single_rrbot_hokuyo_link` appears in `LaserScan.header.frame_id`. A separate system test must verify
that the transform is available from the configured RViz fixed frame.

Keep these claims separate:

| Claim | Appropriate check |
|---|---|
| The node publishes or subscribes to `/tf` | NoDL endpoint description and future conformance |
| The laser frame connects to the robot frame tree | TF lookup in a running system |
| RViz can render the scan | Visual or headless application test |

## 5. Register the curated document

The tutorial verification package registers the document during its build. Confirm that the package builds:

```bash
colcon build --packages-select nodl_tutorial_verification
```

Registration makes the expected document discoverable by package and executable identity. Current `main` does not yet
provide a `ros2 nodl` verb to print or locate a registered document. That lookup is another CLI gap exposed by the
tutorial.

## 6. Current verified flow

The workflow available on `main` is:

1. Start the existing robot.
2. Inventory its nodes.
3. Observe each node into a separate raw runtime record.
4. Select `dummy_laser` as the worked node.
5. Inspect stable application fields.
6. Curate, validate, and register its NoDL document.
7. Verify laser messages and the required transform separately.
8. Finish with the complete working robot in RViz.

This flow is useful for evaluating the demo, but it still has manual translation and comparison steps.

## 7. Target ROSCon flow

The intended complete flow is:

1. Start from the existing `dummy_laser` node.
2. Describe it into a valid draft NoDL document.
3. Curate descriptions and author-only policy.
4. Generate or bind a NoDL-forward interface while retaining scan logic.
5. Compose reusable node and TF endpoint fragments into their correct node documents.
6. Run the original and NoDL-forward variants.
7. Compare expected and observed interfaces.
8. Introduce a deliberate topic, type, parameter, or QoS regression.
9. Show a semantic diff and conformance failure.
10. Restore the correct variant and finish in RViz.

Steps 2 and 4 through 9 require NoDL capabilities that do not exist on `main` yet.

## 8. CLI capabilities exposed by the tutorial

The workflow identifies these required verbs or verb behaviors:

| Need | Candidate command | Available on `main` |
|---|---|---|
| Observe one node as raw graph data | `ros2 nodl describe NODE --raw` | Observation exists; `--raw` spelling does not |
| Describe one node as draft NoDL | `ros2 nodl describe NODE -o FILE.nodl.yaml` | No |
| Validate authored or curated NoDL | `ros2 nodl validate FILE` | Yes |
| Find a registered document | `ros2 nodl show PACKAGE/EXECUTABLE` | No |
| Flatten reusable node fragments | `ros2 nodl compose FILE` | No |
| Generate interface bindings | `ros2 nodl generate FILE` | No |
| Compare two NoDL interfaces | `ros2 nodl diff EXPECTED ACTUAL` | No |
| Check a running node | `ros2 nodl conform NODE --file EXPECTED` | No |

The command names are candidate UX, not accepted interfaces. This tutorial intentionally avoids a `describe-system`
requirement. It keeps node descriptions separate and leaves system membership to launch or a future manifest.

## 9. Suggested regression

Change the `scan` publisher reliability while keeping its ROS type unchanged. This produces an interface-level QoS
change without mixing the demonstration with a compile failure. The future conformance output should identify the
publisher and the changed reliability field.

## 10. Why work through `dummy_laser`

- It has one clear application endpoint.
- Its behavior remains meaningful after interface migration.
- Its output participates in the robot's TF-dependent visualization.
- A topic or QoS regression is easy to explain.
- The demo can recover to a visually working state.
