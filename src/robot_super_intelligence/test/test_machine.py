"""Transition, deadline, and cleanup behavior without ROS dependencies."""

import pytest

from robot_super_intelligence.machine import Machine, STATE_TYPES
from robot_super_intelligence.outcomes import Outcome
from robot_super_intelligence.states import (
    DumpMaterial, Excavate, RunEnd, RunStart,
    TraverseToConstruction, TraverseToExcavation,
)


class FakeState:
    def __init__(self, outcome):
        self.outcome = outcome
        self.outcomes = frozenset({outcome})
        self.entries = 0
        self.canceled = False

    def enter(self):
        self.entries += 1

    def start(self, dispatch):
        return self.outcome

    def cancel(self):
        self.canceled = True


def simulated_states():
    outcomes = (Outcome.START_SUCCESS, Outcome.CONSTRUCTION_REACHED,
                Outcome.DUMP_SUCCESS, Outcome.EXCAVATION_REACHED,
                Outcome.EXCAVATE_SUCCESS, Outcome.RUN_END)
    return {cls: FakeState(outcome) for cls, outcome in zip(STATE_TYPES, outcomes)}


@pytest.mark.parametrize('limit', [0, -1, float('nan'), float('inf')])
def test_invalid_deadlines(limit):
    with pytest.raises(ValueError):
        Machine(limit)


def test_complete_cycle_loops_and_reenters_construction():
    states = simulated_states()
    machine = Machine(100, states, clock=lambda: 0)
    for expected in (TraverseToConstruction, DumpMaterial,
                     TraverseToExcavation, Excavate, TraverseToConstruction):
        machine.advance(machine.state.outcome)
        assert machine.current is expected
    assert machine.completed_cycles == 1
    assert machine.latest_outcome == 'EXCAVATE_SUCCESS'
    assert states[TraverseToConstruction].entries == 2
    assert not machine.done


def test_real_placeholder_aborts_instead_of_simulating_success():
    machine = Machine(100, clock=lambda: 0)
    machine.advance(Outcome.START_SUCCESS)
    machine.advance(Outcome.NOT_IMPLEMENTED)
    assert machine.current is RunEnd
    assert machine.result is Outcome.NOT_IMPLEMENTED
    assert machine.completed_cycles == 0
    machine.advance(Outcome.RUN_END)
    assert machine.done


def test_deadline_interrupts_running_state_and_executes_run_end():
    states = simulated_states()
    machine = Machine(5, states, clock=lambda: 0)
    machine.end(Outcome.TIME_LIMIT_REACHED)
    assert states[RunStart].canceled
    assert machine.current is RunEnd
    assert machine.result is Outcome.TIME_LIMIT_REACHED
    machine.advance(Outcome.RUN_END)
    assert machine.done


@pytest.mark.parametrize('outcome', [Outcome.CANCELED, Outcome.MODE_EXITED,
                                     Outcome.FAILED, Outcome.SHUTDOWN])
def test_interrupt_cleans_up_and_preserves_termination_reason(outcome):
    states = simulated_states()
    machine = Machine(10, states, clock=lambda: 0)
    machine.end(outcome)
    machine.end(Outcome.TIME_LIMIT_REACHED)
    machine.advance(machine.state.outcome)
    assert states[RunStart].canceled
    assert machine.result is outcome
    assert machine.done


def test_undeclared_outcome_is_rejected():
    states = simulated_states()
    machine = Machine(10, states, clock=lambda: 0)
    states[RunStart].outcome = Outcome.DUMP_SUCCESS
    with pytest.raises(ValueError, match='undeclared outcome'):
        machine.advance(machine.state.outcome)


def test_cleanup_exception_still_transitions_to_run_end():
    states = simulated_states()
    def fail_cleanup():
        raise RuntimeError('cleanup failed')
    states[RunStart].cancel = fail_cleanup
    machine = Machine(10, states, clock=lambda: 0)
    with pytest.raises(RuntimeError):
        machine.end(Outcome.FAILED)
    assert machine.current is RunEnd
    assert machine.result is Outcome.FAILED
