# MCAP Fixture Layout

Each fixture is a single `.mcap` file containing four channels (topics), one per
scenario node: `s1_node`, `s2_node`, `s3_node_a`, `s3_node_b`.  Every channel
carries a single CDR-serialised `rosgraph_msgs/msg/Node` message.

## Resolver order (most-specific first)

```
fixtures/<distro>_<rmw>.mcap     — distro + RMW specific override
fixtures/<rmw>.mcap              — RMW-inherent gap (all distros)
fixtures/base.mcap               — full canonical set (covers most combos)
```

All fixtures live flat in this directory (no per-distro subfolders); the distro
override just prefixes the RMW name, e.g. `jazzy_rmw_fastrtps_cpp.mcap`.  The
test resolves the first path that exists.  After regenerating, a maintainer may
promote the new file to a less-specific name and delete the more-specific copy
if the content is identical across distros or RMWs.

The current set (from jazzy) is `base.mcap` (= the zenoh observation, the full
canonical), `rmw_cyclonedds_cpp.mcap` (cyclonedds reports a `KEEP_ALL` queue's
depth as 0), and `jazzy_rmw_fastrtps_cpp.mcap` (jazzy's older fastrtps drops
history/depth over discovery).

## Bootstrapping a new fixture

Run the integration tests with the environment variable set:

```bash
REGEN_FIXTURES=1 colcon test --packages-select nodl_observe \
    --ctest-args -R test_observe_integration
```

(`nodl_observe` is an `ament_cmake` package, so its pytest runs via ctest —
select it with `--ctest-args -R`, not `--pytest-args`. All four scenario nodes
must run in one pass: the regen buffers them and writes the file on the last.)

The file is written to `fixtures/<ROS_DISTRO>_<RMW_IMPLEMENTATION>.mcap`.
Inspect it with the helper script before committing:

```bash
python test/mcap_fixtures.py print fixtures/<distro>_<rmw>.mcap          # YAML
python test/mcap_fixtures.py print fixtures/<distro>_<rmw>.mcap -f json   # JSON
python test/mcap_fixtures.py diff  fixtures/base.mcap fixtures/<distro>_<rmw>.mcap
```

## Dependencies

The `mcap` Python package is required for the fixture helpers and tests.
Install it separately — it is not in the ROS apt index:

```bash
pip install mcap
```
