# SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
# SPDX-License-Identifier: Apache-2.0
"""MCAP fixture helpers: write, read, print, and diff scenario-node fixtures.

Each fixture file is a single MCAP with one channel per scenario node.  The
channel topic is the node basename (``s1_node``, ``s2_node``, ``s3_node_a``,
``s3_node_b``).  Messages are CDR-serialised ``rosgraph_msgs/msg/Node`` payloads.

Human-readable CLI
------------------
Run directly as a script::

    python mcap_fixtures.py print <file.mcap> [channel] [-f yaml|json]  # one/all nodes
    python mcap_fixtures.py diff  <a.mcap> <b.mcap>                     # field diff

Dependencies
------------
* ``mcap`` (pip-installable; not in any ROS apt index -- install separately):
    pip install mcap
  The ``mcap`` and ``rclpy`` imports are guarded so ``--help`` works on a plain
  host without ROS.
"""

# ---------------------------------------------------------------------------
# Guarded imports -- mcap / rclpy absent → graceful help, hard fail on use
# ---------------------------------------------------------------------------

try:
    import mcap  # noqa: F401 (presence check)
    from mcap.reader import make_reader
    from mcap.writer import Writer as McapWriter

    _MCAP_AVAILABLE = True
except ImportError:
    _MCAP_AVAILABLE = False

try:
    import rclpy.serialization as _rclpy_ser

    _RCLPY_AVAILABLE = True
except ImportError:
    _RCLPY_AVAILABLE = False

# Schema encoding for ros2msg (content is best-effort; not used for decoding).
_SCHEMA_ENCODING = 'ros2msg'
_MSG_ENCODING = 'cdr'
_SCHEMA_NAME = 'rosgraph_msgs/msg/Node'

# Top-level field listing of rosgraph_msgs/msg/Node for the schema record.  The
# reader ignores it (we deserialise with the real Python type); it is recorded
# only so external tools (Foxglove, mcap CLI) can identify the message type.  Only
# the top-level fields are listed -- a full ros2msg schema would inline every
# nested type, which is unnecessary for identification.
_NODE_MSG_TEXT = b"""\
# rosgraph_msgs/msg/Node (top-level fields only)
string name
rcl_interfaces/msg/ParameterDescriptor[] parameters
rcl_interfaces/msg/ParameterValue[] parameter_values
rosgraph_msgs/msg/Topic[] publishers
rosgraph_msgs/msg/Topic[] subscriptions
rosgraph_msgs/msg/Service[] service_clients
rosgraph_msgs/msg/Service[] service_servers
rosgraph_msgs/msg/Action[] action_clients
rosgraph_msgs/msg/Action[] action_servers
"""


def _require_mcap():
    if not _MCAP_AVAILABLE:
        raise RuntimeError('mcap is not installed.  Run: pip install mcap')


def _require_rclpy():
    if not _RCLPY_AVAILABLE:
        raise RuntimeError('rclpy is not available (not running inside a sourced ROS workspace)')


# ---------------------------------------------------------------------------
# Core IO
# ---------------------------------------------------------------------------


def write_fixture(path: str, nodes: 'dict[str, object]') -> None:
    """Write *nodes* (``{basename: rosgraph_msgs/msg/Node}``) to an MCAP file.

    Each node is stored as its own channel (topic = basename, e.g. ``s1_node``).
    Payloads are CDR-serialised with ``rclpy.serialization``.
    """
    _require_mcap()
    _require_rclpy()

    with open(path, 'wb') as fh:
        writer = McapWriter(fh)
        writer.start()

        # One registered schema shared across all channels (same message type).
        schema_id = writer.register_schema(
            name=_SCHEMA_NAME,
            encoding=_SCHEMA_ENCODING,
            data=_NODE_MSG_TEXT,
        )

        for basename, node_msg in nodes.items():
            channel_id = writer.register_channel(
                topic=basename,
                message_encoding=_MSG_ENCODING,
                schema_id=schema_id,
            )
            payload = _rclpy_ser.serialize_message(node_msg)
            writer.add_message(
                channel_id=channel_id,
                log_time=0,
                data=payload,
                publish_time=0,
            )

        writer.finish()


def read_fixture(path: str) -> 'dict[str, object]':
    """Read an MCAP fixture and return ``{basename: rosgraph_msgs/msg/Node}``.

    Deserialises each channel's single message with the known Python type
    (``rosgraph_msgs.msg.Node``); the schema record in the file is not used.
    """
    _require_mcap()
    _require_rclpy()

    # Import here so the module can be imported without ROS for --help.
    from rosgraph_msgs.msg import Node as NodeMsg  # noqa: PLC0415

    nodes = {}
    with open(path, 'rb') as fh:
        reader = make_reader(fh)
        for schema, channel, message in reader.iter_messages():
            basename = channel.topic
            nodes[basename] = _rclpy_ser.deserialize_message(message.data, NodeMsg)
    return nodes


# ---------------------------------------------------------------------------
# Rendering + diffing
# ---------------------------------------------------------------------------


def node_to_yaml(node_msg) -> str:
    """Render a ``rosgraph_msgs/msg/Node`` to YAML via rosidl_runtime_py."""
    from rosidl_runtime_py import message_to_yaml  # noqa: PLC0415

    return message_to_yaml(node_msg)


def node_to_json(node_msg) -> str:
    """Render a ``rosgraph_msgs/msg/Node`` to indented JSON (matches the verb's -o)."""
    import json  # noqa: PLC0415

    from rosidl_runtime_py.convert import message_to_ordereddict  # noqa: PLC0415

    return json.dumps(message_to_ordereddict(node_msg), indent=2) + '\n'


def node_to_text(node_msg, fmt: str) -> str:
    """Render a node as ``'yaml'`` or ``'json'``."""
    return node_to_json(node_msg) if fmt == 'json' else node_to_yaml(node_msg)


def field_diff(a, b) -> str:
    """Return a path-qualified diff of the divergent leaves between *a* and *b*.

    Uses ``rosidl_runtime_py.convert.message_to_ordereddict`` to convert both
    messages to nested ordered dicts, then walks the tree and reports only the
    leaves where the values differ.  Returns an empty string when the messages
    are identical.
    """
    from rosidl_runtime_py.convert import message_to_ordereddict  # noqa: PLC0415

    dict_a = message_to_ordereddict(a)
    dict_b = message_to_ordereddict(b)

    lines = []
    _diff_walk(dict_a, dict_b, path='', lines=lines)
    return '\n'.join(lines)


def _diff_walk(a, b, path: str, lines: list) -> None:
    """Recursively walk *a* and *b*, appending divergent leaf lines to *lines*."""
    if type(a) != type(b):  # noqa: E721 (exact type check intentional)
        lines.append(f'{path or "<root>"}:')
        lines.append(f'  - {a!r}')
        lines.append(f'  + {b!r}')
        return

    if isinstance(a, dict):
        all_keys = sorted(set(list(a.keys()) + list(b.keys())))
        for key in all_keys:
            child_path = f'{path}.{key}' if path else str(key)
            if key not in a:
                lines.append(f'{child_path}: <missing in a>  + {b[key]!r}')
            elif key not in b:
                lines.append(f'{child_path}: {a[key]!r}  - <missing in b>')
            else:
                _diff_walk(a[key], b[key], child_path, lines)

    elif isinstance(a, (list, tuple)):
        if len(a) != len(b):
            lines.append(f'{path or "<root>"}: length {len(a)} vs {len(b)}')
            # Still walk up to the shorter length to surface element diffs.
        for i, (ea, eb) in enumerate(zip(a, b)):
            _diff_walk(ea, eb, f'{path}[{i}]', lines)

    else:
        # Leaf scalar.
        if a != b:
            lines.append(f'{path or "<root>"}:')
            lines.append(f'  - {a!r}')
            lines.append(f'  + {b!r}')


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _cli_print(args):
    _require_mcap()
    _require_rclpy()

    nodes = read_fixture(args.file)
    if args.channel:
        if args.channel not in nodes:
            raise SystemExit(f'Channel {args.channel!r} not in fixture.  Available: {sorted(nodes)}')
        print(f'=== {args.channel} ===')
        print(node_to_text(nodes[args.channel], args.format))
    else:
        for basename in sorted(nodes):
            print(f'=== {basename} ===')
            print(node_to_text(nodes[basename], args.format))


def _cli_diff(args):
    _require_mcap()
    _require_rclpy()

    nodes_a = read_fixture(args.a)
    nodes_b = read_fixture(args.b)
    all_channels = sorted(set(list(nodes_a.keys()) + list(nodes_b.keys())))
    any_diff = False
    for ch in all_channels:
        if ch not in nodes_a:
            print(f'--- {ch}: only in {args.b}')
            any_diff = True
            continue
        if ch not in nodes_b:
            print(f'--- {ch}: only in {args.a}')
            any_diff = True
            continue
        diff = field_diff(nodes_a[ch], nodes_b[ch])
        if diff:
            print(f'=== {ch} ===')
            print(diff)
            any_diff = True
    if not any_diff:
        print('(no differences)')


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Print or diff nodl_observe MCAP fixture files.')
    sub = parser.add_subparsers(dest='command', required=True)

    p_print = sub.add_parser('print', help='Print node(s) from a fixture as YAML or JSON')
    p_print.add_argument('file', help='Path to .mcap fixture')
    p_print.add_argument('channel', nargs='?', help='Channel (basename) to print; omit for all')
    p_print.add_argument(
        '-f', '--format', choices=('yaml', 'json'), default='yaml', help='Output format (default: yaml)'
    )
    p_print.set_defaults(func=_cli_print)

    p_diff = sub.add_parser('diff', help='Field-level diff between two fixtures')
    p_diff.add_argument('a', help='First .mcap fixture')
    p_diff.add_argument('b', help='Second .mcap fixture')
    p_diff.set_defaults(func=_cli_diff)

    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
