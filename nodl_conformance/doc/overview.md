# nodl_conformance

`nodl_conformance` is a small Python library that compares two loaded NoDL
documents. It does not load files, inspect running ROS nodes, or provide launch
integration.

```python
from nodl_conformance import diff

differences = diff(expected, actual, node_fqn='/robot/my_node')
```

`expected` and `actual` are `nodl_schema.NodlDocument` objects. An empty list
means conformance. Each `Difference` contains `kind`, `section`, `name`, and
`detail`. Supported kinds are `missing`, `extra`, `type_mismatch`,
`qos_mismatch`, `property_mismatch`, and `unverifiable`.

## Comparison rules

### Interfaces

Interfaces are the public communication endpoints exposed or consumed by a
node.

| Check | Rule |
|---|---|
| Membership | Missing declared interfaces and extra actual interfaces are differences. |
| Collections | Publishers, subscriptions, service servers, service clients, action servers, and action clients remain separate. |
| Names | `/status` stays absolute, `status` resolves in the node namespace, and `~/status` resolves below the full node name. |
| Types | A short type equals its kind-specific fully qualified form. For example, `std_msgs/String` equals `std_msgs/msg/String` for a topic. |
| Representation | Collection order and empty collections do not affect the result. |
| Ignored metadata | Description fields and top-level `codegen` metadata do not affect the result. |

### QoS

Quality of Service (QoS) policies define how an endpoint communicates.

| Check | Rule |
|---|---|
| Observability | Declared QoS that is not observable produces an `unverifiable` difference. |
| Optional policies | An omitted expected policy places no requirement on the actual endpoint. `SYSTEM_DEFAULT` and `BEST_AVAILABLE` accept a concrete actual policy. |
| Concrete policies | An unknown actual policy is `unverifiable`. Otherwise, the actual policy must match. |
| History and depth | `KEEP_LAST` requires a matching depth. Other history policies ignore depth. |
| Durations | Omitted and zero durations both mean unlimited. Finite nonzero durations must match exactly. |

### Parameters

Parameters are named configuration entries exposed by a node.

| Check | Rule |
|---|---|
| Membership | Missing declared parameters and extra actual parameters are differences. |
| Type | Types must match. A generic actual type cannot prove a declared fixed-size type and produces `unverifiable`. |
| Read-only | A declared `read_only` value must match. An unknown actual value is `unverifiable`. |
| Ignored fields | Description, default value, additional constraints, validation rules, and the actual current value do not affect the result. |

## Runtime and package-test integration

`ros2nodl` loads and composes the expected document, describes a live node, and
calls this comparator through `ros2 nodl conform`.
The small `ament_nodl_conformance` package provides
optional CMake and launch-test integration for ROS packages.
