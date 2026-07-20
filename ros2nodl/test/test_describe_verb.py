# SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
# SPDX-License-Identifier: Apache-2.0
"""CLI smoke tests for the ``ros2 nodl describe`` verb.

The verb shells out to the C++ ``observe`` executable (from ``nodl_observe``),
subscribes to the latched ``rosgraph_msgs/Node`` it publishes, and renders it.
The smoke tests below therefore require a live ROS environment *and* the built
``observe`` binary; they skip automatically where either is absent (e.g. the
macOS dev machine).  The third-party "latched publish reaches a late subscriber"
semantics belong to the binary and are covered by ``nodl_observe``'s own
integration test -- here we test the verb's own responsibilities: it drives the
binary, renders YAML/JSON, relays exit codes, and honours its flags.

Design notes:

- Tests drive the verb through its Python API (:meth:`DescribeVerb.main`) to
  stay consistent with ``test_verbs.py``'s style.
- A lightweight target node is spun up *in-process* in an isolated ROS domain
  to avoid interference with stray nodes on the machine.
- Each test shares one :mod:`rclpy` init/shutdown via the ``ros_context``
  fixture; the verb detects the already-initialised context and does not own it.
"""

import argparse
import json
import os

import pytest

# Guard: skip the whole module if the ROS stack is absent.  (nodl_observe is now
# a C++ package -- it is NOT importable as Python, so we must not importorskip it;
# the smoke tests instead gate on the built `observe` binary below.)
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('rosgraph_msgs')

import yaml  # noqa: E402
from rclpy.qos import ReliabilityPolicy  # noqa: E402

from ros2nodl.verb.describe import DescribeVerb, _infer_format, _observe_binary  # noqa: E402

# Live observation requires Iron+ (REP-2011 type hashes / int32-safe QoS
# durations); pre-Iron distros (Humble) are a tracked follow-up.  The
# pure-argument tests below still run everywhere; only the smoke tests that
# actually observe a node are gated.  BEST_AVAILABLE presence is the Iron+ proxy.
_IRON_PLUS = hasattr(ReliabilityPolicy, 'BEST_AVAILABLE')

# The smoke tests need the built `observe` executable on the install space.
_BINARY = _observe_binary()
_SMOKE_OK = _IRON_PLUS and _BINARY is not None
_SMOKE_REASON = (
    'live observation requires Iron+' if not _IRON_PLUS else 'the nodl_observe `observe` binary is not built/installed'
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_DOMAIN_ID = int(os.environ.get('ROS_DOMAIN_ID', '42'))
_TARGET_NODE = '/ros2nodl_test_target'


def _make_args(**kwargs):
    """Build a minimal :class:`argparse.Namespace` for :meth:`DescribeVerb.main`."""
    defaults = dict(
        node_name=_TARGET_NODE,
        timeout=5.0,
        no_params=False,
        topic='/nodl/observed_node_test',
        output=None,
    )
    defaults.update(kwargs)
    return argparse.Namespace(**defaults)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope='module')
def ros_context():
    """Module-scoped rclpy init/shutdown in an isolated, test-only ROS domain."""
    os.environ.setdefault('ROS_DOMAIN_ID', str(_DOMAIN_ID))
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def target_node(ros_context):
    """A live rclpy node that acts as the observation target.

    Spun on a background executor so the binary (a separate process) discovers
    it and its endpoints over DDS.
    """
    import threading

    import std_msgs.msg  # present in any standard ROS 2 install
    from rclpy.executors import SingleThreadedExecutor

    node = rclpy.create_node(_TARGET_NODE.lstrip('/'))
    node.create_publisher(std_msgs.msg.String, '/test_topic', 10)
    node.create_subscription(std_msgs.msg.String, '/test_topic', lambda _: None, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield node
    finally:
        executor.shutdown()
        thread.join(timeout=3.0)
        node.destroy_node()


# ---------------------------------------------------------------------------
# Pure-Python (no ROS) tests — run anywhere
# ---------------------------------------------------------------------------


class TestInferFormat:
    def test_yaml_extension(self):
        assert _infer_format('out.yaml') == 'yaml'

    def test_yml_extension(self):
        assert _infer_format('out.yml') == 'yaml'

    def test_json_extension(self):
        assert _infer_format('out.json') == 'json'

    def test_uppercase_extension(self):
        assert _infer_format('OUT.YAML') == 'yaml'

    def test_unknown_extension_raises(self):
        import argparse as _ap

        with pytest.raises(_ap.ArgumentTypeError, match='unrecognised extension'):
            _infer_format('out.txt')

    def test_no_extension_raises(self):
        import argparse as _ap

        with pytest.raises(_ap.ArgumentTypeError):
            _infer_format('noextension')


class TestDescribeVerbBadArgs:
    """Tests that do not require a live ROS environment."""

    def test_unknown_output_extension_returns_1(self, capsys):
        verb = DescribeVerb()
        args = _make_args(output='out.txt')
        rc = verb.main(args=args)
        assert rc == 1
        assert 'unrecognised extension' in capsys.readouterr().err

    def test_unknown_output_extension_no_ros_started(self, capsys):
        """Verify the extension check happens *before* any ROS work."""
        verb = DescribeVerb()
        args = _make_args(output='out.png')
        rc = verb.main(args=args)
        assert rc == 1


# ---------------------------------------------------------------------------
# ROS smoke tests — require rclpy + the observe binary + a running target node
# ---------------------------------------------------------------------------


@pytest.mark.skipif(not _SMOKE_OK, reason=_SMOKE_REASON)
class TestDescribeVerbSmoke:
    def test_exit_code_zero(self, target_node):
        """Verb exits 0 when the target node is present."""
        verb = DescribeVerb()
        rc = verb.main(args=_make_args())
        assert rc == 0

    def test_stdout_is_valid_yaml_with_node_name(self, target_node, capsys):
        """Default output (no -o) is YAML that contains the target node's FQN."""
        verb = DescribeVerb()
        verb.main(args=_make_args())
        out = capsys.readouterr().out
        doc = yaml.safe_load(out)
        assert doc is not None, 'stdout did not parse as YAML'
        # rosgraph_msgs/Node has a 'name' field at the top level.
        assert 'name' in doc, f'YAML output missing "name" field: {doc!r}'
        assert _TARGET_NODE in doc['name'], f'Expected {_TARGET_NODE!r} in name field, got {doc["name"]!r}'

    def test_output_json_is_valid(self, target_node, tmp_path):
        """``-o foo.json`` writes parseable JSON."""
        out_file = tmp_path / 'obs.json'
        verb = DescribeVerb()
        rc = verb.main(args=_make_args(output=str(out_file)))
        assert rc == 0
        assert out_file.exists(), '-o did not create the output file'
        data = json.loads(out_file.read_text())
        assert isinstance(data, dict), 'JSON output is not an object'
        assert 'name' in data

    def test_output_yaml_matches_stdout(self, target_node, tmp_path, capsys):
        """``-o foo.yaml`` and stdout (no -o) produce the same bytes."""
        verb = DescribeVerb()
        verb.main(args=_make_args())
        stdout_text = capsys.readouterr().out

        out_file = tmp_path / 'obs.yaml'
        verb.main(args=_make_args(output=str(out_file)))

        assert out_file.read_text() == stdout_text, '-o .yaml output differs from stdout'

    def test_custom_topic_used(self, target_node):
        """``--topic`` overrides the publish destination.

        The verb both publishes (via the binary) and subscribes on the same
        topic to render; a clean exit proves the message flowed over the custom
        topic end to end.
        """
        verb = DescribeVerb()
        rc = verb.main(args=_make_args(topic='/nodl/custom_topic_smoke_test'))
        assert rc == 0

    def test_node_not_found_returns_nonzero(self, ros_context, capsys):
        """Verb returns nonzero with a clear message when the target is absent."""
        verb = DescribeVerb()
        rc = verb.main(args=_make_args(node_name='/nonexistent_node_xyzzy', timeout=2.0))
        assert rc != 0
        err = capsys.readouterr().err
        assert 'not found' in err.lower() or 'nonexistent' in err.lower(), (
            f'Expected a not-found message in stderr, got: {err!r}'
        )

    def test_no_params_flag_succeeds(self, target_node, capsys):
        """``--no-params`` completes successfully and does not contact the target."""
        verb = DescribeVerb()
        rc = verb.main(args=_make_args(no_params=True))
        assert rc == 0
        out = capsys.readouterr().out
        assert yaml.safe_load(out) is not None
