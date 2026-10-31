"""Ownership, priorities, locks, stale commands, and stale grants."""

import pytest

from robot_super_intelligence.arbitration import Arbitration


def test_priority_only_applies_to_explicitly_eligible_sources():
    arbiter = Arbitration({'teleop': 100, 'dig': 50}, clock=lambda: 0)
    arbiter.select(['dig'], False)
    arbiter.receive('drive', 'teleop', 'operator')
    arbiter.receive('drive', 'dig', 'routine')
    assert arbiter.output('drive') == 'routine'
    arbiter.select(['dig', 'teleop'], False)
    arbiter.receive('drive', 'teleop', 'operator')
    arbiter.receive('drive', 'dig', 'routine')
    assert arbiter.output('drive') == 'operator'


def test_grant_change_and_unlock_discard_old_commands():
    arbiter = Arbitration({'dig': 50}, clock=lambda: 0)
    arbiter.select(['dig'], False)
    arbiter.receive('drive', 'dig', 'old')
    arbiter.select(['dig'], False)
    assert arbiter.output('drive') is None


def test_one_lock_inhibits_both_resources():
    arbiter = Arbitration({'dig': 50}, clock=lambda: 0)
    arbiter.select(['dig'], False)
    for channel in ('drive', 'arm_drum'):
        arbiter.receive(channel, 'dig', 'command')
    arbiter.stop_lock = True
    assert arbiter.outputs() == {}


def test_priority_never_mixes_drive_and_arm_sources():
    arbiter = Arbitration({'dig': 50, 'teleop': 100}, clock=lambda: 0)
    arbiter.select(['dig', 'teleop'], False)
    arbiter.receive('drive', 'dig', 'dig-drive')
    arbiter.receive('arm_drum', 'dig', 'dig-arm')
    arbiter.receive('drive', 'teleop', 'teleop-drive')
    assert arbiter.outputs() == {'drive': 'teleop-drive'}


def test_commands_expire_without_falling_back_to_unauthorized_sources():
    now = [0]
    arbiter = Arbitration({'dig': 50, 'teleop': 100}, clock=lambda: now[0])
    arbiter.select(['dig'], False)
    arbiter.receive('drive', 'dig', 'command')
    now[0] = 0.5
    arbiter.receive('drive', 'teleop', 'operator')
    assert arbiter.output('drive') is None
    with pytest.raises(ValueError):
        arbiter.select(['unknown'], False)
