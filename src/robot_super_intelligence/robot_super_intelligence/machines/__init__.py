"""Runnable machine definitions sharing the same execution engine."""

from .full_auto import FullAutoMachine
from .run_start import RunStartMachine
from .dig_traverse_dump import DigTraverseDumpMachine
from .one_cycle import OneCycleMachine


MACHINES = {
    'full_auto': FullAutoMachine,
    'run_start': RunStartMachine,
    'dig_traverse_dump': DigTraverseDumpMachine,
    'one_cycle': OneCycleMachine,
}
