# Tutorials

These tutorials test how NoDL fits real ROS 2 development workflows. The first pass uses nodes from
[`ros2/demos`](https://github.com/ros2/demos) and stays within the capabilities available on NoDL `main`.

```{toctree}
:hidden:

basics
dummy-robot
nav2-controller
```

:::{important}
The **Basics** and **Dummy robot** tutorials have current, executable subsets. The **Nav2 ControllerServer** tutorial
is a target workflow for composition. `nodl_schema` can resolve `include` references, but NoDL does not yet provide
forward generation, semantic diff, or expected-versus-observed conformance. Each tutorial separates commands that
work today from a clearly labelled target workflow.
:::

## Available tutorials

### [ROS 2 basics](basics.md)

Describe, curate, validate, and register the same small publisher interface in C++ and Python.

### [Dummy robot](dummy-robot.md)

Apply the current workflow to a visible robot system, a laser publisher, and TF-related system requirements.

### [Nav2 ControllerServer](nav2-controller.md)

Show how a NoDL document would compose lifecycle, bond, TF, and common Nav2 conventions with a node-specific action
server and topic endpoints.

## Tutorial roadmap

Five tutorials remain the target suite. The table distinguishes implemented documentation from planned work.

| Tutorial | Upstream target | Status | Adoption question |
|---|---|---|---|
| [ROS 2 basics](basics.md) | `ros2/demos`: `demo_nodes_cpp`, `demo_nodes_py` | Prototype available | How does one NoDL interface map to C++ and Python? |
| [Dummy robot](dummy-robot.md) | `ros2/demos`: `dummy_robot` | Prototype available | How does the workflow scale to a visible multi-node robot? |
| Pendulum control | `ros2/demos`: `pendulum_control` | **TBD** | Can NoDL migrate interfaces without owning real-time behavior? |
| ros2_control | `ros2_control_demos`: Examples 1 and 17 | **TBD** | How do framework-provided capabilities compose? |
| [Nav2 ControllerServer](nav2-controller.md) | `navigation2`: `nav2_controller::ControllerServer` | Target workflow | Can composition scale across ecosystem bases? |

<!-- TODO(nodl-tutorials):
Target: ros2/demos/pendulum_control, with separate pendulum_controller and pendulum_motor documents.
Prerequisites: advanced C++ endpoint bindings and logical-node registration for two nodes in one executable.
Proof: preserve RttExecutor, allocator, memory strategies, QoS, scheduling, timers, callbacks, and existing tests.
PR boundary: one ros2/demos implementation PR plus one NoDL tutorial and locked verification update.
-->

<!-- TODO(nodl-tutorials):
Target: ros-controls/ros2_control_demos Examples 1 and 17.
Prerequisites: fragments, logical-node registration, semantic diff, and conformance.
Proof: detect missing /diagnostics, /rrbot/hardware_status, and separate /rrbot_custom_status interfaces.
PR boundary: one ros2_control_demos implementation PR plus one NoDL tutorial and locked verification update.
-->

## Recommended talk path

Use **Dummy robot** for the full ROSCon narrative because RViz makes the result visible. Use **ROS 2 basics** only for
a short syntax and C++/Python introduction. The later framework tutorials are scaling evidence, not extra live demos.

Conformance is not a separate sixth tutorial. Each track will add an intentional regression when the conformance
command is available.
