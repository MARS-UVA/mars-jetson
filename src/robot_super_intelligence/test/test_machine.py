"""Transition, deadline, and cleanup behavior without ROS dependencies."""

import pytest

from robot_super_intelligence.machine import Machine
from robot_super_intelligence.machines import DigTraverseDumpMachine, FullAutoMachine, RunStartMachine
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

    def start(self, dispatch, on_complete):
        on_complete(self.outcome)

    def cancel(self):
        self.canceled = True


def simulated_states():
    outcomes = (Outcome.START_SUCCESS, Outcome.CONSTRUCTION_REACHED,
                Outcome.DUMP_SUCCESS, Outcome.EXCAVATION_REACHED,
                Outcome.EXCAVATE_SUCCESS, Outcome.RUN_END)
    return {cls: FakeState(outcome) for cls, outcome in zip(FullAutoMachine.state_types, outcomes)}


@pytest.mark.parametrize('limit', [0, -1, float('nan'), float('inf')])
@pytest.mark.parametrize('machine_type', [FullAutoMachine, RunStartMachine, DigTraverseDumpMachine])
def test_invalid_deadlines(machine_type, limit):
    with pytest.raises(ValueError):
        machine_type(limit)


def test_complete_cycle_loops_and_reenters_construction():
    states = simulated_states()
    machine = FullAutoMachine(100, states, clock=lambda: 0)
    for expected in (TraverseToConstruction, DumpMaterial,
                     TraverseToExcavation, Excavate, TraverseToConstruction):
        machine.advance(machine.state.outcome)
        assert machine.current is expected
    assert machine.completed_cycles == 1
    assert machine.latest_outcome == 'EXCAVATE_SUCCESS'
    assert states[TraverseToConstruction].entries == 2
    assert not machine.done


def test_real_placeholder_aborts_instead_of_simulating_success():
    machine = FullAutoMachine(100, clock=lambda: 0)
    machine.advance(Outcome.START_SUCCESS)
    machine.advance(Outcome.NOT_IMPLEMENTED)
    assert machine.current is RunEnd
    assert machine.result is Outcome.NOT_IMPLEMENTED
    assert machine.completed_cycles == 0
    machine.advance(Outcome.RUN_END)
    assert machine.done


def test_deadline_interrupts_running_state_and_executes_run_end():
    states = simulated_states()
    machine = FullAutoMachine(5, states, clock=lambda: 0)
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
    machine = FullAutoMachine(10, states, clock=lambda: 0)
    machine.end(outcome)
    machine.end(Outcome.TIME_LIMIT_REACHED)
    machine.advance(machine.state.outcome)
    assert states[RunStart].canceled
    assert machine.result is outcome
    assert machine.done


def test_undeclared_outcome_is_rejected():
    states = simulated_states()
    machine = FullAutoMachine(10, states, clock=lambda: 0)
    states[RunStart].outcome = Outcome.DUMP_SUCCESS
    with pytest.raises(ValueError, match='undeclared outcome'):
        machine.advance(machine.state.outcome)


def test_cleanup_exception_still_transitions_to_run_end():
    states = simulated_states()
    def fail_cleanup():
        raise RuntimeError('cleanup failed')
    states[RunStart].cancel = fail_cleanup
    machine = FullAutoMachine(10, states, clock=lambda: 0)
    with pytest.raises(RuntimeError):
        machine.end(Outcome.FAILED)
    assert machine.current is RunEnd
    assert machine.result is Outcome.FAILED


def test_run_start_finishes_after_construction_without_entering_dump():
    states = {RunStart: FakeState(Outcome.START_SUCCESS),
              TraverseToConstruction: FakeState(Outcome.CONSTRUCTION_REACHED),
              RunEnd: FakeState(Outcome.RUN_END)}
    machine = RunStartMachine(100, states, clock=lambda: 0)
    machine.advance(Outcome.START_SUCCESS)
    assert machine.current is TraverseToConstruction
    machine.advance(Outcome.CONSTRUCTION_REACHED)
    assert machine.current is RunEnd
    assert not states[TraverseToConstruction].canceled
    machine.advance(Outcome.RUN_END)
    assert machine.done
    assert machine.result is Outcome.CONSTRUCTION_REACHED
    assert machine.result in machine.success_outcomes
    assert machine.completed_cycles == 0


def test_machines_reuse_state_classes_but_have_independent_instances_and_graphs():
    full = FullAutoMachine(100, clock=lambda: 0)
    short = RunStartMachine(100, clock=lambda: 0)
    assert full.states[RunStart] is not short.states[RunStart]
    for machine in (full, short):
        machine.advance(Outcome.START_SUCCESS)
        machine.advance(Outcome.CONSTRUCTION_REACHED)
    assert full.current is DumpMaterial
    assert short.current is RunEnd
    assert not full.done


@pytest.mark.parametrize('machine_type, succeeds', [(FullAutoMachine, True), (RunStartMachine, False)])
def test_each_machine_defines_whether_its_deadline_is_success(machine_type, succeeds):
    machine = machine_type(10, clock=lambda: 0)
    machine.end(Outcome.TIME_LIMIT_REACHED)
    machine.advance(Outcome.RUN_END)
    assert machine.done
    assert (machine.result in machine.success_outcomes) == succeeds


def test_engine_supports_a_different_initial_state():
    class DumpOnlyMachine(Machine):
        state_types = (DumpMaterial, RunEnd)
        initial_state = DumpMaterial
        end_state = RunEnd
        transitions = {(DumpMaterial, Outcome.DUMP_SUCCESS): RunEnd}
        success_outcomes = frozenset({Outcome.DUMP_SUCCESS})
    machine = DumpOnlyMachine(10, clock=lambda: 0)
    assert machine.current is DumpMaterial
    machine.advance(Outcome.DUMP_SUCCESS)
    machine.advance(Outcome.RUN_END)
    assert machine.done
    assert machine.result is Outcome.DUMP_SUCCESS
    assert machine.completed_cycles == 0


def test_dig_traverse_dump_has_explicit_transitions_for_every_state_outcome():
    machine = DigTraverseDumpMachine(10)
    assert machine.current is Excavate
    for state in (Excavate, TraverseToConstruction, DumpMaterial):
        for outcome in machine.states[state].outcomes:
            assert (state, outcome) in machine.transitions
    for outcome, expected in [
        (Outcome.EXCAVATE_SUCCESS, TraverseToConstruction),
        (Outcome.CONSTRUCTION_REACHED, DumpMaterial),
        (Outcome.DUMP_SUCCESS, RunEnd),
    ]:
        machine.advance(outcome)
        assert machine.current is expected
    machine.advance(Outcome.RUN_END)
    assert machine.done
    assert machine.result is Outcome.DUMP_SUCCESS
    assert machine.completed_cycles == 1
