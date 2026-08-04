# ROS 2 basics in C++ and Python

This tutorial applies the current NoDL workflow to the `talker` examples in `demo_nodes_cpp` and `demo_nodes_py`.
Both nodes publish `example_interfaces/msg/String` messages on the relative `chatter` topic. Their queue depths differ:
the C++ example uses 7 and the Python example uses 10.

:::{note}
This is a **prototype tutorial**. Validation, registration, observation, and manual curation work today. Forward
generation, composition, semantic diff, and conformance are target behavior and are not yet implemented.
:::

## What this tutorial demonstrates

- A NoDL document describes an interface, not timer or message-content behavior.
- The same schema describes C++ and Python nodes.
- Source-level relative names remain relative in an authored document.
- Runtime observation can recover endpoint facts, but currently returns a raw `rosgraph_msgs/Node` message.
- Python composition is a design target that still needs a fragment model and generator.

## Conventional implementations

The upstream implementations are:

- [`demo_nodes_cpp/src/topics/talker.cpp`](https://github.com/ros2/demos/blob/f8d20abaec2be76c7062b2a0242ae6f82e2857b8/demo_nodes_cpp/src/topics/talker.cpp)
- [`demo_nodes_py/topics/talker.py`](https://github.com/ros2/demos/blob/f8d20abaec2be76c7062b2a0242ae6f82e2857b8/demo_nodes_py/demo_nodes_py/topics/talker.py)

Run the C++ node:

```bash
ros2 run demo_nodes_cpp talker
```

In another terminal, inspect its graph interface:

```bash
ros2 nodl describe /talker --no-params -o /tmp/cpp_talker.observed.yaml
```

Stop the C++ node before starting the Python node because both use the name `/talker`.

```bash
ros2 run demo_nodes_py talker
```

In another terminal:

```bash
ros2 nodl describe /talker --no-params -o /tmp/py_talker.observed.yaml
```

:::{warning}
The two output files above serialize `rosgraph_msgs/Node`. They are observation records, not NoDL documents, and
`ros2 nodl validate` will correctly reject them.
:::

## Curated NoDL documents

The C++ source creates a depth-7 publisher:

```{literalinclude} ../../../examples/nodl_tutorial_verification/nodl/cpp_talker.nodl.yaml
:language: yaml
```

The Python source creates a depth-10 publisher:

```{literalinclude} ../../../examples/nodl_tutorial_verification/nodl/py_talker.nodl.yaml
:language: yaml
```

Validate both documents:

```bash
ros2 nodl validate \
  examples/nodl_tutorial_verification/nodl/cpp_talker.nodl.yaml \
  examples/nodl_tutorial_verification/nodl/py_talker.nodl.yaml
```

The timer period and `Hello World` text do not appear in NoDL. They are behavior, not ROS interface declarations.

## What can be compared today

Current observation can confirm these stable application facts:

| Field | C++ | Python |
|---|---|---|
| Node name | `/talker` | `/talker` |
| Publisher | `/chatter` after ROS name resolution | `/chatter` after ROS name resolution |
| Type | `example_interfaces/msg/String` | `example_interfaces/msg/String` |
| History | `KEEP_LAST` | `KEEP_LAST` |
| Depth | 7 | 10 |

This is a raw-field inspection. It is not semantic NoDL conformance. Middleware discovery can also report some QoS
fields as unknown, so an unknown observed value must not be replaced with an assumed value.

## Target: Python composition

The Python example should eventually accept a composed document with a reusable `rclpy` base and an application
publisher capability. A later design may resemble the following shape:

```yaml
# Design preview only. This is not valid NoDL v2 syntax today.
nodl_version: 2
fragments:
  - nodl://python/rclpy_node
  - nodl://tutorials/string_talker
```

The resolved document should drive a Python implementation while user code continues to own its timer and message
contents. The same application fragment should also be usable by a C++ binding where its interface is equivalent.

This target needs:

1. A public fragment syntax and resolver.
2. Python forward generation or runtime binding.
3. Semantic normalization across relative and resolved names.
4. Expected-versus-observed conformance.

## Target: deliberate regression

When semantic conformance exists, change the Python publisher depth or reliability. The expected result is a
path-qualified QoS mismatch, followed by a successful comparison after restoration.

Services, actions, constrained parameters, and callbacks will extend this tutorial after the first workflow is
accepted.

## CLI capabilities exposed by the tutorial

The small example identifies the same missing boundary with less system context:

| Need | Candidate command | Available on `main` |
|---|---|---|
| Describe a node as valid NoDL | `ros2 nodl describe NODE -o FILE.nodl.yaml` | No |
| Validate the curated document | `ros2 nodl validate FILE` | Yes |
| Resolve Python capability fragments | `ros2 nodl compose FILE` | No |
| Generate C++ or Python bindings | `ros2 nodl generate FILE` | No |
| Compare language implementations | `ros2 nodl diff CPP PYTHON` | No |
| Check a running implementation | `ros2 nodl conform NODE --file EXPECTED` | No |

These names are candidate UX. The tutorial's purpose is to make the required operations concrete before their public
contracts are fixed.
