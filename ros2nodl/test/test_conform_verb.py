# SPDX-FileCopyrightText: 2026 Open Source Robotics Foundation, Inc.
# SPDX-License-Identifier: Apache-2.0

import argparse
from pathlib import Path
from unittest.mock import Mock

from ros2nodl.verb.conform import ConformVerb


def _args(**overrides):
    values = {
        'node_name': '/robot/controller',
        'file': Path('nodl/controller.nodl.yaml'),
        'timeout': 15.0,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_conform_passes_runtime_arguments(monkeypatch, capsys):
    calls = []

    def assert_conforms(**kwargs):
        calls.append(kwargs)

    import ros2nodl.conformance

    monkeypatch.setattr(ros2nodl.conformance, 'assert_conforms', assert_conforms)

    assert ConformVerb().main(args=_args(timeout=2.5)) == 0
    assert calls == [
        {
            'nodl_file': 'nodl/controller.nodl.yaml',
            'node_fqn': '/robot/controller',
            'timeout_sec': 2.5,
        }
    ]
    assert capsys.readouterr().out == '/robot/controller: conforms\n'


def test_conform_reports_all_differences(monkeypatch, capsys):
    message = "NoDL conformance failed for '/robot/controller':\n  [missing] publishers '/state': not observed"

    import ros2nodl.conformance

    monkeypatch.setattr(
        ros2nodl.conformance,
        'assert_conforms',
        Mock(side_effect=AssertionError(message)),
    )

    assert ConformVerb().main(args=_args()) == 1
    error = capsys.readouterr().err
    assert error.startswith('ros2 nodl conform: NoDL conformance failed')
    assert "[missing] publishers '/state': not observed" in error


def test_conform_reports_runtime_errors(monkeypatch, capsys):
    import ros2nodl.conformance

    monkeypatch.setattr(
        ros2nodl.conformance,
        'assert_conforms',
        Mock(side_effect=ValueError('failed to load document')),
    )

    assert ConformVerb().main(args=_args()) == 1
    assert capsys.readouterr().err == 'ros2 nodl conform: failed to load document\n'


def test_conform_arguments_are_required_and_typed():
    parser = argparse.ArgumentParser()
    ConformVerb().add_arguments(parser, 'ros2 nodl conform')

    args = parser.parse_args(['/robot/controller', '--file', 'node.nodl.yaml', '--timeout', '3.5'])

    assert args.node_name == '/robot/controller'
    assert args.file == Path('node.nodl.yaml')
    assert args.timeout == 3.5
