# SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Layer-2 observation integration tests: scenario graphs -> MCAP fixtures.

Each scenario spins a *dummy ROS environment* in-process (scenario nodes on a
background executor), then drives the C++ ``observe`` binary as a subprocess for
each target FQN.  The binary latched-publishes the resulting
``rosgraph_msgs/msg/Node`` on ``/nodl/observed_node``; we receive it in-process
via a matching transient-local subscription.

The live observation is compared **field-exact** against the committed MCAP
fixture resolved for the active ``(ROS_DISTRO, RMW_IMPLEMENTATION)`` pair.

Isolation
---------
Stray nodes on the host must not leak into the observed graph.  The scenarios
run under ``ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`` and a per-run, *derived*
(not random) ``ROS_DOMAIN_ID``, with ``RMW_IMPLEMENTATION`` defaulting to
``rmw_fastrtps_cpp`` if unset.  Nothing in the fixture content depends on the
domain id or hostname; a dedicated test asserts that.

Fixture resolver (most-specific first)
---------------------------------------
``test/fixtures/<distro>/<rmw>.mcap``   -- distro + RMW specific
``test/fixtures/<rmw>.mcap``            -- RMW-inherent gap (all distros)
``test/fixtures/_base.mcap``            -- full canonical set (most combos)

Fixture regeneration
--------------------
Set ``REGEN_FIXTURES=1`` to (re)write the most-specific fixture
``test/fixtures/<distro>/<rmw>.mcap`` instead of comparing.  The maintainer is
expected to inspect the regenerated file before committing; after inspection the
file can be promoted to a less-specific path (and the old one deleted) if the
values are identical across distros or RMWs.
"""

import os
import subprocess
import sys
import threading
import time

import pytest

# mcap_fixtures lives in the same directory; use an explicit path-insert so this
# works under ament_add_pytest_test, which runs the file directly.
sys.path.insert(0, os.path.dirname(__file__))

# Guard: skip if the ROS stack is absent (e.g. macOS host) or if mcap is not
# installed (graceful skip -- the mcap import is not in any ROS apt index and
# must be installed separately with `pip install mcap`).
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mcap', reason=('mcap is not installed; run: pip install mcap'))

# Guard: skip if rosgraph_msgs.msg.Node is absent (pre-2.0.4 distros).
try:
    from rosgraph_msgs.msg import Node as _NodeMsg  # noqa: E402
except ImportError:
    pytest.skip('rosgraph_msgs.msg.Node is unavailable on this distro', allow_module_level=True)

import mcap_fixtures  # noqa: E402  (after path-insert + importorskip)


def _isolate_ros_env():
    """Pin the RMW + an isolated, derived domain before rclpy is initialised."""
    # Derived (deterministic per process), NOT random -- but fixture content
    # must not depend on it, which a separate test asserts.
    domain_id = str(os.getpid() % 100 + 100)  # 100..199, away from common ids
    os.environ.setdefault('ROS_DOMAIN_ID', domain_id)
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')


# Must run before rclpy.init so the colcon runner picks up the env regardless of
# invocation order.
_isolate_ros_env()

from example_interfaces.action import Fibonacci  # noqa: E402
from example_interfaces.srv import AddTwoInts  # noqa: E402
from rcl_interfaces.msg import ParameterDescriptor  # noqa: E402
from rclpy.action import ActionClient, ActionServer  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import QoSProfile as _QoSMsg  # noqa: E402
from std_msgs.msg import String  # noqa: E402

# REP-2011 topic type hashes and the BEST_AVAILABLE QoS enum are Iron+.  Humble
# observes the same Node message with those fields simply left unset (best-effort
# fill, plan #2) -- it is NOT skipped; its fixture captures the unset values.
# BEST_AVAILABLE presence on the rclpy enum is the Iron+ proxy.
_IRON_PLUS = hasattr(ReliabilityPolicy, 'BEST_AVAILABLE')

# ---------------------------------------------------------------------------
# Fixture resolver
# ---------------------------------------------------------------------------

_ROS_DISTRO = os.environ.get('ROS_DISTRO', 'unknown')
_RMW = os.environ.get('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
_FIXTURES_DIR = os.path.join(os.path.dirname(__file__), 'fixtures')
_REGEN_PATH = os.path.join(_FIXTURES_DIR, f'{_ROS_DISTRO}_{_RMW}.mcap')
_REGEN = os.environ.get('REGEN_FIXTURES') == '1'
_OBSERVE_TIMEOUT = 15.0  # seconds to wait for the binary's latched publish


def _fixture_candidates():
    """Candidate fixture paths for the active (distro, RMW), most-specific first."""
    return (
        os.path.join(_FIXTURES_DIR, f'{_ROS_DISTRO}_{_RMW}.mcap'),  # distro+RMW
        os.path.join(_FIXTURES_DIR, f'{_RMW}.mcap'),  # RMW (any distro)
        os.path.join(_FIXTURES_DIR, 'base.mcap'),  # canonical base
    )


def _resolve_fixture():
    """Return the most-specific existing fixture path, or ``None``."""
    for path in _fixture_candidates():
        if os.path.exists(path):
            return path
    return None


# Bootstrap guard: if no fixture resolves and we are not regenerating, skip the
# whole module with a hint so the maintainer knows what to do.
if not _REGEN and _resolve_fixture() is None:
    pytest.skip(
        f'no MCAP fixture for ROS distro {_ROS_DISTRO!r} / RMW {_RMW!r} '
        f'(looked at: {", ".join(_fixture_candidates())}); '
        'run with REGEN_FIXTURES=1 to bootstrap a fixture',
        allow_module_level=True,
    )

# ---------------------------------------------------------------------------
# Session-scoped rclpy init / shutdown
# ---------------------------------------------------------------------------


def _start_zenoh_router():
    """Start rmw_zenohd (the per-RMW ctest sets RMW_IMPLEMENTATION but starts no
    router) and return the Popen so the session fixture can tear it down."""
    import glob  # noqa: PLC0415
    import shutil  # noqa: PLC0415

    exe = shutil.which('rmw_zenohd')
    if exe is None:
        distro = os.environ.get('ROS_DISTRO', '')
        matches = glob.glob(f'/opt/ros/{distro}/lib/rmw_zenoh_cpp/rmw_zenohd') or glob.glob(
            '/opt/ros/*/lib/rmw_zenoh_cpp/rmw_zenohd'
        )
        exe = matches[0] if matches else None
    if exe is None:
        raise RuntimeError('RMW_IMPLEMENTATION=rmw_zenoh_cpp but rmw_zenohd was not found')
    proc = subprocess.Popen([exe], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3.0)  # allow the router to come up before rclpy.init()
    return proc


@pytest.fixture(scope='session', autouse=True)
def _ros_session():
    """Init/shutdown rclpy once for the module; own the zenoh router when needed."""
    router = _start_zenoh_router() if _RMW == 'rmw_zenoh_cpp' else None
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()
        if router is not None:
            router.terminate()
            try:
                router.wait(timeout=5)
            except subprocess.TimeoutExpired:
                router.kill()
                router.wait()


# ---------------------------------------------------------------------------
# Scenario graph builders
# ---------------------------------------------------------------------------


def _build_s1(_ctx):
    """S1 minimal: 1 node, 1 pub, 1 sub, all default QoS."""
    node = Node('s1_node', namespace='/s1')
    node.create_publisher(String, 'chatter', 10)
    node.create_subscription(String, 'commands', lambda _m: None, 10)
    return [node]


def _build_s2(_ctx):
    """S2 full-surface: every endpoint kind, QoS deliberately varied."""
    node = Node('s2_node', namespace='/s2')

    # 3 parameters, one read-only.
    node.declare_parameter('speed', 1.5)
    node.declare_parameter('max_count', 10)
    node.declare_parameter('mode', 'auto', ParameterDescriptor(description='operating mode', read_only=True))

    # Topics with deliberately varied QoS across the enum space.
    node.create_publisher(
        String,
        'be_pub',
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST),
    )
    node.create_publisher(
        String,
        'tl_pub',
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
        ),
    )
    node.create_subscription(String, 'dl_sub', lambda _m: None, QoSProfile(depth=5, deadline=Duration(seconds=2)))

    # Service server + client with explicit non-default QoS.  The fixture must
    # still show *_UNKNOWN -- service QoS is not observable (the documented
    # can't-observe limitation, locked in as a test).
    non_default_srv_qos = QoSProfile(
        depth=3,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_ALL,
    )
    node.create_service(AddTwoInts, 'add', lambda _req, resp: resp, qos_profile=non_default_srv_qos)
    node.create_client(AddTwoInts, 'multiply', qos_profile=non_default_srv_qos)

    # Action server + client -> their _action/* constituents must be folded.
    ActionServer(node, Fibonacci, 'fib', lambda goal: goal)
    ActionClient(node, Fibonacci, 'compute')
    return [node]


def _build_s3(_ctx):
    """S3 multi-node isolation: A (target) and B share a topic with different QoS."""
    node_a = Node('node_a', namespace='/s3')
    node_b = Node('node_b', namespace='/s3')

    # A and B both touch /s3/shared with QoS that differs, exercising attribution.
    node_a.create_publisher(
        String,
        '/s3/shared',
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST),
    )
    node_b.create_subscription(
        String,
        '/s3/shared',
        lambda _m: None,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST),
    )

    # A also has a private subscription; B has its own service + action.
    node_a.create_subscription(String, '/s3/a_only', lambda _m: None, 10)
    node_b.create_service(AddTwoInts, 'b_add', lambda _req, resp: resp)
    ActionServer(node_b, Fibonacci, 'b_fib', lambda goal: goal)
    return [node_a, node_b]


# Each scenario: builder + the (target_fqn, channel_basename) pairs to observe.
_SCENARIOS = {
    's1': (_build_s1, [('/s1/s1_node', 's1_node')]),
    's2': (_build_s2, [('/s2/s2_node', 's2_node')]),
    's3': (
        _build_s3,
        [
            ('/s3/node_a', 's3_node_a'),
            ('/s3/node_b', 's3_node_b'),
        ],
    ),
}

# ---------------------------------------------------------------------------
# Scenario graph context manager
# ---------------------------------------------------------------------------


class _ScenarioGraph:
    """Spins scenario nodes on a background executor."""

    def __init__(self, builder):
        self._builder = builder
        self.nodes = []
        self._executor = None
        self._thread = None

    def __enter__(self):
        self.nodes = self._builder(None)
        self._executor = SingleThreadedExecutor()
        for node in self.nodes:
            self._executor.add_node(node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._executor.shutdown()
        self._thread.join(timeout=3.0)
        for node in self.nodes:
            node.destroy_node()
        return False


# ---------------------------------------------------------------------------
# observe binary + in-process subscription
# ---------------------------------------------------------------------------


def _observe_binary_path():
    """Return the installed path of the ``observe`` executable."""
    from ament_index_python.packages import get_package_prefix  # noqa: PLC0415

    prefix = get_package_prefix('nodl_observe')
    return os.path.join(prefix, 'lib', 'nodl_observe', 'observe')


# Latched QoS matching the binary's /nodl/observed_node publisher.
_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def _receive_observed_node(timeout_sec: float) -> '_NodeMsg':
    """Create a transient-local sub-node, wait for one message, destroy it.

    The binary latched-publishes on ``/nodl/observed_node``; the subscription
    uses matching QoS so it replays the latched message even after the binary
    has finished publishing.
    """
    received = []
    event = threading.Event()

    receiver = Node('_nodl_observe_test_receiver')
    try:
        receiver.create_subscription(
            _NodeMsg,
            '/nodl/observed_node',
            lambda msg: (received.append(msg), event.set()),
            _LATCHED_QOS,
        )
        # Drive the receiver node in a temporary single-threaded executor.
        exec_ = SingleThreadedExecutor()
        exec_.add_node(receiver)
        deadline = time.monotonic() + timeout_sec
        while not event.is_set() and time.monotonic() < deadline:
            exec_.spin_once(timeout_sec=0.1)
        exec_.shutdown()
    finally:
        receiver.destroy_node()

    if not received:
        raise TimeoutError(f'no message on /nodl/observed_node within {timeout_sec}s')
    return received[0]


def _observe_target(target_fqn: str) -> '_NodeMsg':
    """Run the ``observe`` binary for *target_fqn* and return the Node message.

    The binary is given ``--spin-seconds 5`` so it stays up long enough for the
    latched message to be delivered, then exits on its own.  We kill it on
    timeout regardless.
    """
    binary = _observe_binary_path()
    proc = subprocess.Popen(
        [binary, target_fqn, '--spin-seconds', '5'],
        env=os.environ.copy(),
    )
    try:
        node_msg = _receive_observed_node(timeout_sec=_OBSERVE_TIMEOUT)
    finally:
        # Always reap the subprocess.
        try:
            proc.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
    return node_msg


# ---------------------------------------------------------------------------
# Session-scoped observation cache
# ---------------------------------------------------------------------------


_OBSERVE_CACHE: 'dict[str, dict[str, _NodeMsg]]' = {}


def _observe_scenario(scenario: str) -> 'dict[str, _NodeMsg]':
    """Observe all targets in *scenario*, returning ``{basename: NodeMsg}``."""
    builder, targets = _SCENARIOS[scenario]
    results = {}
    with _ScenarioGraph(builder):
        # Brief settle time so discovery propagates before observing.
        time.sleep(0.5)
        for target_fqn, basename in targets:
            results[basename] = _observe_target(target_fqn)
    return results


def _observed(scenario: str) -> 'dict[str, _NodeMsg]':
    """Return cached observations for *scenario*, observing on first call."""
    if scenario not in _OBSERVE_CACHE:
        _OBSERVE_CACHE[scenario] = _observe_scenario(scenario)
    return _OBSERVE_CACHE[scenario]


# ---------------------------------------------------------------------------
# Determinism guards (run on live messages before any fixture comparison)
# ---------------------------------------------------------------------------


def _string_fields(node_msg) -> 'list[str]':
    """Return all string scalar values reachable in *node_msg* via ordereddict.

    Numeric scalars (hash bytes, QoS integers) are excluded so the leak check
    focuses on human-meaningful text fields only.
    """
    from rosidl_runtime_py.convert import message_to_ordereddict  # noqa: PLC0415

    data = message_to_ordereddict(node_msg)

    result = []

    def walk(obj):
        if isinstance(obj, str):
            result.append(obj)
        elif isinstance(obj, dict):
            for k, v in obj.items():
                result.append(k)
                walk(v)
        elif isinstance(obj, (list, tuple)):
            for item in obj:
                walk(item)

    walk(data)
    return result


def _assert_deterministic(basename: str, node_msg) -> None:
    """Assert that *node_msg* contains no domain-id or hostname leakage."""
    import socket  # noqa: PLC0415

    domain_id = os.environ.get('ROS_DOMAIN_ID', '')
    hostname = socket.gethostname()
    strings = _string_fields(node_msg)
    if domain_id:
        assert not any(domain_id in s for s in strings), (
            f'{basename}: domain id {domain_id!r} leaked into a string field'
        )
    assert not any(hostname in s for s in strings), f'{basename}: hostname {hostname!r} leaked into a string field'


def _assert_arrays_sorted(node_msg) -> None:
    """Assert all endpoint arrays on *node_msg* are sorted by name."""
    assert [p.name for p in node_msg.publishers] == sorted(p.name for p in node_msg.publishers), 'publishers not sorted'
    assert [s.name for s in node_msg.subscriptions] == sorted(s.name for s in node_msg.subscriptions), (
        'subscriptions not sorted'
    )
    assert [s.name for s in node_msg.service_servers] == sorted(s.name for s in node_msg.service_servers), (
        'service_servers not sorted'
    )
    assert [s.name for s in node_msg.service_clients] == sorted(s.name for s in node_msg.service_clients), (
        'service_clients not sorted'
    )


# ---------------------------------------------------------------------------
# Fixture comparison tests
# ---------------------------------------------------------------------------


_ALL_TARGETS = [(scenario, basename) for scenario, (_b, targets) in _SCENARIOS.items() for _fqn, basename in targets]


@pytest.mark.parametrize('scenario,basename', _ALL_TARGETS)
def test_observation_matches_fixture(scenario, basename):
    """Observed Node matches the committed MCAP fixture (or regenerates it)."""
    observed = _observed(scenario)[basename]

    # Run determinism guards on the live observation before comparing/saving.
    _assert_deterministic(basename, observed)
    if basename == 's2_node':
        _assert_arrays_sorted(observed)

    if _REGEN:
        # Collect all four nodes in one pass so the MCAP is written atomically.
        # Individual test invocations may be parametrized; buffer until the last
        # one.  We collect into a module-level dict and flush after the final
        # parametrize iteration.
        _REGEN_BUFFER[basename] = observed
        if set(_REGEN_BUFFER.keys()) >= {b for _s, b in _ALL_TARGETS}:
            os.makedirs(os.path.dirname(_REGEN_PATH), exist_ok=True)
            mcap_fixtures.write_fixture(_REGEN_PATH, _REGEN_BUFFER)
            pytest.skip(f'REGEN_FIXTURES: wrote {_REGEN_PATH}')
        else:
            pytest.skip(f'REGEN_FIXTURES: buffered {basename}, waiting for rest')

    # Normal comparison path.
    fixture_path = _resolve_fixture()
    assert fixture_path is not None, (
        f'No MCAP fixture for {basename!r} '
        f'(distro {_ROS_DISTRO!r}, RMW {_RMW!r}). '
        'Run with REGEN_FIXTURES=1 to generate it.'
    )

    fixture_nodes = mcap_fixtures.read_fixture(fixture_path)
    assert basename in fixture_nodes, (
        f'Channel {basename!r} is missing from fixture {fixture_path!r}. '
        'Run with REGEN_FIXTURES=1 to regenerate the fixture.'
    )

    diff = mcap_fixtures.field_diff(fixture_nodes[basename], observed)
    assert not diff, (
        f'Observation of {basename!r} drifted from fixture {fixture_path!r}.\n'
        f'{diff}\n'
        'Re-run with REGEN_FIXTURES=1 if this change is intended.'
    )


# Buffer for REGEN mode: accumulate all four nodes then flush.
_REGEN_BUFFER: 'dict[str, object]' = {}

# ---------------------------------------------------------------------------
# Targeted structural assertions (each traces to an observer failure mode)
# ---------------------------------------------------------------------------


def _find(endpoints, name):
    """Return the endpoint named *name*, listing candidates on miss."""
    for endpoint in endpoints:
        if endpoint.name == name:
            return endpoint
    raise AssertionError(f'no endpoint named {name!r}; present: {sorted(e.name for e in endpoints)}')


def _live(basename: str):
    """Return the live-observed Node for *basename* (from cache)."""
    for scenario, (_b, targets) in _SCENARIOS.items():
        for _fqn, name in targets:
            if name == basename:
                return _observed(scenario)[basename]
    raise KeyError(basename)


class TestS1Minimal:
    """Baseline: implicit endpoints present + unfiltered, type hash correct."""

    def test_user_topics_present_with_type_hash(self):
        msg = _live('s1_node')
        chatter = _find(msg.publishers, '/s1/chatter')
        assert chatter.type.name == 'std_msgs/msg/String'
        if _IRON_PLUS:
            # Iron+: a populated RIHS type hash, not all-zero.
            assert any(b != 0 for b in bytes(chatter.type.hash.value))
        else:
            # Humble: no REP-2011 type hash -> left unset (all-zero), honest-unknown.
            assert all(b == 0 for b in bytes(chatter.type.hash.value))
        sub = _find(msg.subscriptions, '/s1/commands')
        assert sub.type.name == 'std_msgs/msg/String'

    def test_implicit_endpoints_unfiltered(self):
        msg = _live('s1_node')
        pub_names = {p.name for p in msg.publishers}
        srv_names = {s.name for s in msg.service_servers}
        # /rosout + /parameter_events stay flat and unfiltered (Observe records).
        assert '/rosout' in pub_names
        assert '/parameter_events' in pub_names
        # The implicit parameter services are all present.
        for suffix in (
            'list_parameters',
            'describe_parameters',
            'get_parameters',
            'set_parameters',
            'set_parameters_atomically',
            'get_parameter_types',
        ):
            assert f'/s1/s1_node/{suffix}' in srv_names


class TestS2FullSurface:
    """Every field exercised; topic QoS across the enum space; service UNKNOWN."""

    def test_varied_topic_qos_captured(self):
        msg = _live('s2_node')
        be = _find(msg.publishers, '/s2/be_pub')
        assert be.qos.reliability == _QoSMsg.RELIABILITY_BEST_EFFORT
        tl = _find(msg.publishers, '/s2/tl_pub')
        # Reliability and durability propagate on every (distro, RMW); history
        # and depth do NOT on some combinations -- captured by the fixture, not
        # re-asserted here.
        assert tl.qos.durability == _QoSMsg.DURABILITY_TRANSIENT_LOCAL
        dl = _find(msg.subscriptions, '/s2/dl_sub')
        assert dl.qos.deadline.sec == 2

    def test_service_qos_unknown_despite_non_default_wire_qos(self):
        # The documented can't-observe limitation, locked in: services were
        # created with explicit non-default QoS, yet the observed QoS is UNKNOWN.
        msg = _live('s2_node')
        add = _find(msg.service_servers, '/s2/add')
        assert add.request_qos.reliability == _QoSMsg.RELIABILITY_UNKNOWN
        assert add.request_qos.durability == _QoSMsg.DURABILITY_UNKNOWN
        mul = _find(msg.service_clients, '/s2/multiply')
        assert mul.response_qos.reliability == _QoSMsg.RELIABILITY_UNKNOWN

    def test_actions_folded_not_flat(self):
        msg = _live('s2_node')
        server = _find(msg.action_servers, '/s2/fib')
        assert server.send_goal.name == '/s2/fib/_action/send_goal'
        assert server.feedback.name == '/s2/fib/_action/feedback'
        client = _find(msg.action_clients, '/s2/compute')
        assert client.get_result.name == '/s2/compute/_action/get_result'
        # No _action/* constituent may leak into a flat list.
        flat = (
            [p.name for p in msg.publishers]
            + [s.name for s in msg.subscriptions]
            + [s.name for s in msg.service_servers]
            + [s.name for s in msg.service_clients]
        )
        assert not any('/_action/' in n for n in flat)

    def test_parameter_descriptors_and_values(self):
        msg = _live('s2_node')
        by_name = {d.name: (d, v) for d, v in zip(msg.parameters, msg.parameter_values)}
        assert 'speed' in by_name and 'max_count' in by_name and 'mode' in by_name
        mode_desc, _ = by_name['mode']
        assert mode_desc.read_only is True
        _, speed_val = by_name['speed']
        assert abs(speed_val.double_value - 1.5) < 1e-9


class TestS3Isolation:
    """By-node attribution + QoS cross-attribution on a shared topic."""

    def test_target_a_does_not_leak_node_b(self):
        msg = _live('s3_node_a')
        flat = (
            [p.name for p in msg.publishers]
            + [s.name for s in msg.subscriptions]
            + [s.name for s in msg.service_servers]
        )
        # B's private endpoints must not appear when observing A.
        assert '/s3/a_only' in flat
        assert '/s3/b_add' not in flat
        assert all('b_fib' not in n for n in flat)
        assert not msg.action_servers, "A has no actions; B's must not leak in"

    def test_shared_topic_carries_targets_own_qos(self):
        # A publishes /s3/shared RELIABLE; B subscribes BEST_EFFORT.  Each
        # fixture channel must carry its *own* node's QoS, never the peer's.
        msg_a = _live('s3_node_a')
        shared_a = _find(msg_a.publishers, '/s3/shared')
        assert shared_a.qos.reliability == _QoSMsg.RELIABILITY_RELIABLE
        msg_b = _live('s3_node_b')
        shared_b = _find(msg_b.subscriptions, '/s3/shared')
        assert shared_b.qos.reliability == _QoSMsg.RELIABILITY_BEST_EFFORT


class TestDeterminism:
    """Verify that no run-specific values leak into the observed messages."""

    def test_no_domain_id_or_hostname_leaked(self):
        """No string-valued field in any observed node encodes domain id or hostname."""
        for scenario in _SCENARIOS:
            for basename in _observed(scenario):
                _assert_deterministic(basename, _observed(scenario)[basename])

    def test_endpoint_arrays_sorted(self):
        """observe binary sorts every endpoint array (name ascending)."""
        _assert_arrays_sorted(_live('s2_node'))
