"""Completion callbacks are queued, consumed once, and ignored after stopping."""

from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock

from robot_super_intelligence.machine_runner import MachineRunner
from robot_super_intelligence.machines import FullAutoMachine, RunStartMachine
from robot_super_intelligence.outcomes import Outcome
from robot_super_intelligence.states import RunEnd, RunStart, TraverseToConstruction


class Executor:
    def __init__(self):
        self.pending = deque()

    def create_task(self, callback, *args):
        self.pending.append((callback, args))

    def step(self):
        callback, args = self.pending.popleft()
        callback(*args)

    def drain(self):
        for _ in range(50):
            if not self.pending:
                return
            self.step()
        raise AssertionError('A finite machine did not finish')


class State:
    def __init__(self, outcome, waiting=False, duplicate=False):
        self.outcome = outcome
        self.outcomes = frozenset({outcome})
        self.waiting = waiting
        self.duplicate = duplicate
        self.canceled = False

    def enter(self):
        pass

    def start(self, dispatch, on_complete):
        self.on_complete = on_complete
        if not self.waiting:
            on_complete(self.outcome)
            if self.duplicate:
                on_complete(Outcome.FAILED)

    def cancel(self):
        self.canceled = True


def setup_runner(waiting=False, duplicate=False):
    states = {RunStart: State(Outcome.START_SUCCESS),
              TraverseToConstruction: State(Outcome.CONSTRUCTION_REACHED, waiting, duplicate),
              RunEnd: State(Outcome.RUN_END)}
    executor = Executor()
    node = SimpleNamespace(executor=executor, get_logger=lambda: Mock())
    runner = MachineRunner(node, RunStartMachine(10, states), Mock(), Mock())
    return runner, executor, states


def test_immediate_states_complete_through_callbacks():
    runner, executor, _ = setup_runner()
    completed = Mock()
    assert runner.start(completed) is None
    completed.assert_not_called()
    executor.drain()
    completed.assert_called_once_with(Outcome.CONSTRUCTION_REACHED)
    assert runner.done


def test_duplicate_state_callback_does_not_advance_the_next_state():
    runner, executor, states = setup_runner(duplicate=True)
    completed = Mock()
    runner.start(completed)
    executor.drain()
    completed.assert_called_once_with(Outcome.CONSTRUCTION_REACHED)
    states[TraverseToConstruction].on_complete(Outcome.FAILED)
    executor.drain()
    completed.assert_called_once()


def test_late_callback_after_cancel_is_ignored():
    runner, executor, states = setup_runner(waiting=True)
    completed = Mock()
    runner.start(completed)
    executor.drain()
    runner.stop(Outcome.CANCELED)
    states[TraverseToConstruction].on_complete(Outcome.CONSTRUCTION_REACHED)
    runner.stop(Outcome.ESTOP)
    executor.drain()
    assert states[TraverseToConstruction].canceled
    completed.assert_called_once_with(Outcome.CANCELED)


def test_immediate_loop_does_not_recurse():
    from robot_super_intelligence.states import DumpMaterial, Excavate, TraverseToExcavation
    states = {RunStart: State(Outcome.START_SUCCESS),
              TraverseToConstruction: State(Outcome.CONSTRUCTION_REACHED),
              DumpMaterial: State(Outcome.DUMP_SUCCESS),
              TraverseToExcavation: State(Outcome.EXCAVATION_REACHED),
              Excavate: State(Outcome.EXCAVATE_SUCCESS),
              RunEnd: State(Outcome.RUN_END)}
    executor = Executor()
    node = SimpleNamespace(executor=executor, get_logger=lambda: Mock())
    runner = MachineRunner(node, FullAutoMachine(10, states), Mock(), Mock())
    completed = Mock()
    runner.start(completed)
    for _ in range(5000):
        executor.step()
    assert runner.machine.completed_cycles > 100
    completed.assert_not_called()
    runner.stop(Outcome.TIME_LIMIT_REACHED)
    executor.drain()
    completed.assert_called_once_with(Outcome.TIME_LIMIT_REACHED)


def test_cleanup_exception_still_reports_completion_once():
    runner, executor, states = setup_runner(waiting=True)
    completed = Mock()
    runner.start(completed)
    executor.drain()
    states[TraverseToConstruction].cancel = Mock(side_effect=RuntimeError('cleanup failed'))
    runner.stop(Outcome.ESTOP)
    completed.assert_called_once_with(Outcome.ESTOP)
    assert runner.done
