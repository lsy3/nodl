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

Comparison is strict. Every declared public interface must be present in the
actual document, and every actual public interface must be declared.

| Check | Rule |
|---|---|
| **Interfaces · membership** | Missing declared interfaces and extra actual interfaces are differences. |
| **Interfaces · collections** | Publishers, subscriptions, service servers, service clients, action servers, and action clients remain separate. |
| **Interfaces · names** | `/status` stays absolute, `status` resolves in the node namespace, and `~/status` resolves below the full node name. |
| **Interfaces · types** | A short type equals its kind-specific fully qualified form. For example, `std_msgs/String` equals `std_msgs/msg/String` for a topic. |
| **QoS · observability** | Declared QoS that is not observable produces an `unverifiable` difference. |
| **QoS · optional policies** | An omitted expected policy places no requirement on the actual endpoint. `SYSTEM_DEFAULT` and `BEST_AVAILABLE` accept a concrete actual policy. |
| **QoS · concrete policies** | An unknown actual policy is `unverifiable`. Otherwise, the actual policy must match. |
| **QoS · history and depth** | `KEEP_LAST` requires a matching depth. Other history policies ignore depth. |
| **QoS · durations** | Omitted and zero durations both mean unlimited. Finite nonzero durations must match exactly. |
| **Parameters · membership** | Missing declared parameters and extra actual parameters are differences. |
| **Parameters · type** | Types must match. A generic actual type cannot prove a declared fixed-size type and produces `unverifiable`. |
| **Parameters · read-only** | A declared `read_only` value must match. An unknown actual value is `unverifiable`. |
| **Parameters · ignored fields** | Description, default value, additional constraints, validation rules, and the actual current value do not affect the result. |
| **Representation** | Collection order and empty collections do not affect the result. |
| **Metadata** | Description fields and top-level `codegen` metadata do not affect the result. |

## Runtime integration

`ros2nodl` loads and composes the expected document, describes a live node, and
calls this comparator through `ros2 nodl conform`.
