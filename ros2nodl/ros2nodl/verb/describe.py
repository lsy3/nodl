# SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
# SPDX-License-Identifier: Apache-2.0
"""``ros2 nodl describe NODE_NAME [knobs...]`` -- observe a running node and publish its description.

The observation itself is performed by the C++ ``observe`` executable (from the
``nodl_observe`` package): the verb is a thin wrapper that shells out to it,
receives the latched ``rosgraph_msgs/Node`` the binary publishes, and renders it.
The serialized message is the only language boundary -- there is no in-process
C++<->Python message conversion and no pybind build surface.
"""

import argparse
import json
import os
import subprocess
import sys
import time

from ros2nodl.verb import VerbExtension

_DEFAULT_TOPIC = '/nodl/observed_node'
_DEFAULT_TIMEOUT = 5.0

# How long the binary is asked to stay alive after publishing, so its latched
# (transient_local) sample is still being served when we subscribe to render it.
_KEEPALIVE_SEC = 3.0


def _infer_format(path: str) -> str:
    """Return 'yaml' or 'json' inferred from *path*'s extension.

    Raises :class:`argparse.ArgumentTypeError` for any other extension so the
    caller can surface it as an argparse-level error before ROS is initialised.
    """
    _, ext = os.path.splitext(path)
    ext = ext.lower()
    if ext in ('.yaml', '.yml'):
        return 'yaml'
    if ext == '.json':
        return 'json'
    raise argparse.ArgumentTypeError(f'-o/--output: unrecognised extension "{ext}"; use .yaml, .yml, or .json')


class DescribeVerb(VerbExtension):
    """Observe a running node and publish its description as rosgraph_msgs/Node."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'node_name',
            metavar='NODE_NAME',
            help='Fully-qualified name of the target node (e.g. /my_namespace/my_node).',
        )
        parser.add_argument(
            '--timeout',
            metavar='SEC',
            type=float,
            default=_DEFAULT_TIMEOUT,
            help=(
                'Maximum time in seconds to wait for discovery, parameter services, '
                'and publish acknowledgement (default: %(default)s).'
            ),
        )
        parser.add_argument(
            '--no-params',
            action='store_true',
            default=False,
            dest='no_params',
            help=('Skip remote parameter service calls. Faster and zero-contact with the target node.'),
        )
        parser.add_argument(
            '--topic',
            metavar='NAME',
            default=_DEFAULT_TOPIC,
            help='Latched topic the description is published on (default: %(default)s).',
        )
        parser.add_argument(
            '-o',
            '--output',
            metavar='FILE',
            default=None,
            dest='output',
            help=(
                'Write the description to FILE instead of stdout. '
                'Format is inferred from the extension: .yaml/.yml or .json.'
            ),
        )

    def main(self, *, args):
        # Validate the output extension before touching ROS so the error is clean.
        output_format = None
        if args.output is not None:
            try:
                output_format = _infer_format(args.output)
            except argparse.ArgumentTypeError as e:
                print(str(e), file=sys.stderr)
                return 1

        return _run(
            node_name=args.node_name,
            timeout_sec=args.timeout,
            include_parameters=not args.no_params,
            topic=args.topic,
            output_path=args.output,
            output_format=output_format,
        )


def _observe_binary():
    """Return the path to the C++ ``observe`` executable, or ``None`` if absent."""
    try:
        from ament_index_python.packages import get_package_prefix

        prefix = get_package_prefix('nodl_observe')
    except Exception:
        return None
    candidate = os.path.join(prefix, 'lib', 'nodl_observe', 'observe')
    return candidate if os.path.isfile(candidate) else None


def _latched_qos():
    """The QoS the ``observe`` binary latch-publishes with (reliable, transient_local, depth 1)."""
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _to_yaml(msg) -> str:
    from rosidl_runtime_py import message_to_yaml

    return message_to_yaml(msg)


def _to_json(msg) -> str:
    from rosidl_runtime_py.convert import message_to_ordereddict

    return json.dumps(message_to_ordereddict(msg), indent=2) + '\n'


def _run(
    *,
    node_name: str,
    timeout_sec: float,
    include_parameters: bool,
    topic: str,
    output_path,
    output_format,
) -> int:
    binary = _observe_binary()
    if binary is None:
        print(
            'ros2 nodl describe: the nodl_observe `observe` executable was not '
            'found; build/install the nodl_observe package (it provides the '
            'observation backend).',
            file=sys.stderr,
        )
        return 1

    import rclpy
    from rosgraph_msgs.msg import Node as NodeMsg

    # Spawn the observer: it observes the target, latch-publishes on `topic`, and
    # stays alive for `_KEEPALIVE_SEC` so this process can subscribe and render.
    cmd = [
        binary,
        node_name,
        '--timeout',
        repr(float(timeout_sec)),
        '--topic',
        topic,
        '--spin-seconds',
        repr(_KEEPALIVE_SEC),
    ]
    if not include_parameters:
        cmd.append('--no-parameters')
    proc = subprocess.Popen(cmd, stderr=subprocess.PIPE, text=True)

    # Own the rclpy lifecycle only if nobody initialised it yet (an embedding
    # caller -- e.g. an in-process test harness -- that already called
    # rclpy.init() keeps ownership).
    try:
        rclpy.init()
        owns_context = True
    except RuntimeError:
        owns_context = False

    received = []
    try:
        node = rclpy.create_node(f'_ros2nodl_describe_{os.getpid()}', start_parameter_services=False)
        try:
            node.create_subscription(NodeMsg, topic, lambda m: received.append(m), _latched_qos())

            # The binary may spend up to `timeout_sec` discovering before it
            # publishes; wait for that plus the keepalive window plus margin.
            deadline = time.monotonic() + timeout_sec + _KEEPALIVE_SEC + 2.0
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if not received and proc.poll() is not None:
                    # The binary exited before publishing -- usually node-not-found.
                    break

            if not received:
                _, err = proc.communicate(timeout=2.0)
                if err:
                    sys.stderr.write(err if err.endswith('\n') else err + '\n')
                rc = proc.returncode if proc.returncode not in (None, 0) else 1
                if rc == 1 and not err:
                    print(
                        f'ros2 nodl describe: timed out waiting for an observation of {node_name!r} on {topic!r}.',
                        file=sys.stderr,
                    )
                return rc

            msg = received[0]
            if output_path is None:
                print(_to_yaml(msg), end='')
            else:
                text = _to_json(msg) if output_format == 'json' else _to_yaml(msg)
                try:
                    with open(output_path, 'w') as fh:
                        fh.write(text)
                except OSError as e:
                    print(f'ros2 nodl describe: {e}', file=sys.stderr)
                    return 1
        finally:
            node.destroy_node()
    finally:
        if owns_context:
            rclpy.shutdown()
        # Reap the observer; it should exit on its own after the keepalive window.
        try:
            proc.wait(timeout=_KEEPALIVE_SEC + 2.0)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

    return 0
