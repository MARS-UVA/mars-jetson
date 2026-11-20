"""Runnable machine definitions sharing the same execution engine."""

from .full_auto import FullAutoMachine
from .run_start import RunStartMachine
from .dig_traverse_dump import DigTraverseDumpMachine
from .one_cycle import OneCycleMachine
from .start_berm_test import StartBermTestMachine
from .localize_test import LocalizeTestMachine


MACHINES = {
    'full_auto': FullAutoMachine,
    'run_start': RunStartMachine,
    'dig_traverse_dump': DigTraverseDumpMachine,
    'one_cycle': OneCycleMachine,
    'start_berm_test': StartBermTestMachine,
    'localize_test': LocalizeTestMachine,
}
