# MCAP Fixture Layout

Each `.mcap` has four channels (`s1_node`, `s2_node`, `s3_node_a`, `s3_node_b`),
one CDR `rosgraph_msgs/msg/Node` each.

## Resolver (most-specific first)

```
fixtures/<distro>_<rmw>.mcap   distro+RMW override
fixtures/<rmw>.mcap            RMW-inherent gap (all distros)
fixtures/base.mcap             canonical set (most combos)
```

Files are flat here. Current set (from jazzy): `base.mcap` (zenoh, canonical),
`rmw_cyclonedds_cpp.mcap` (cyclonedds reports a `KEEP_ALL` depth as 0),
`jazzy_rmw_fastrtps_cpp.mcap` (old fastrtps drops history/depth over discovery).

## Regenerate

```bash
REGEN_FIXTURES=1 colcon test --packages-select nodl_observe \
    --ctest-args -R test_observe_integration
```

Writes `fixtures/<distro>_<rmw>.mcap` (all four scenarios run in one pass). Inspect
before committing:

```bash
python test/mcap_fixtures.py print fixtures/<f>.mcap        # add -f json for JSON
python test/mcap_fixtures.py diff fixtures/base.mcap fixtures/<f>.mcap
```

Fixtures are read via the `mcap` package (pulled in by the `mcap_ros2_support` test
dep) + `rclpy.serialization` for CDR — not the rosbag2 storage API.
