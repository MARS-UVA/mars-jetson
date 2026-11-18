"""Bruno's selected-end dump sequence and its action server."""

from autonomy_msgs.action import Dump
from robot_control_msgs.msg import ArmControlMode, RobotState

from digdump_actions.server import Command, RoutineServer, run


DEFAULTS = {'dump_speed': 1.0, 'drive_speed': 0.25, 'dump_time': 6.0, 'move_time': 6.0}


class BrunoDump:
    """Drive toward the selected end, stop, then reverse that drum."""

    def __init__(self, parameters, back_selected=False):
        self.p = parameters
        self.back_selected = back_selected
        self.phase = 'driving'
        self.started = None

    def step(self, now, positions=None):
        if self.started is None:
            self.started = now
        if self.phase == 'driving':
            if now - self.started < self.p['move_time']:
                direction = -1.0 if self.back_selected else 1.0
                return self.phase, Command(drive=direction * self.p['drive_speed']), False
            self.phase = 'dumping'
            self.started = None
            return self.phase, Command(), False

        if now - self.started < self.p['dump_time']:
            command = Command()
            if self.back_selected:
                command.back_drum = -self.p['dump_speed']
            else:
                command.front_drum = -self.p['dump_speed']
            return self.phase, command, False
        return self.phase, Command(), True

class FrontloaderDump:
    """Reverse into the zone, raise the back actuator, then dump regolith."""

    DRIVE_SPEED = 0.25
    DRIVE_TIME = 12.0
    RAISE_SPEED = 0.5
    RAISE_TIME = 2.0
    DUMP_SPEED = 1.0
    DUMP_TIME = 6.0

    def __init__(self, _parameters, back_selected=False):
        self.phase = 'driving'
        self.started = None

    def step(self, now, positions=None):
        if self.started is None:
            self.started = now

        if self.phase == 'driving':
            if now - self.started < self.DRIVE_TIME:
                return self.phase, Command(drive=-self.DRIVE_SPEED), False
            self.phase = 'raising'
            self.started = now
            return self.phase, Command(), False

        if self.phase == 'raising':
            if now - self.started < self.RAISE_TIME:
                return self.phase, Command(back_arm=self.RAISE_SPEED), False
            self.phase = 'dumping'
            self.started = now
            return self.phase, Command(), False

        if now - self.started < self.DUMP_TIME:
            return self.phase, Command(back_drum=-self.DUMP_SPEED), False
        return self.phase, Command(), True


class GroundDrumDump:
    """Drive into the zone, then dump while slowly reversing."""

    ENTER_SPEED = 0.25
    ENTER_TIME = 12.0
    REVERSE_SPEED = 0.1
    DUMP_SPEED = 0.75
    DUMP_TIME = 6.0

    def __init__(self, _parameters, back_selected=False):
        self.phase = 'entering'
        self.started = None

    def step(self, now, positions=None):
        if self.started is None:
            self.started = now

        if self.phase == 'entering':
            if now - self.started < self.ENTER_TIME:
                return self.phase, Command(drive=self.ENTER_SPEED), False
            self.phase = 'dumping'
            self.started = now
            return self.phase, Command(), False

        if now - self.started < self.DUMP_TIME:
            return self.phase, Command(
                front_drum=-self.DUMP_SPEED,
                back_drum=-self.DUMP_SPEED,
                drive=-self.REVERSE_SPEED,
            ), False
        return self.phase, Command(), True

class DumpNode(RoutineServer):
    def __init__(self):
        super().__init__('dump', Dump, RobotState.DUMP, DEFAULTS,
                         {Dump.Goal.BRUNO_INDEX: BrunoDump,
                          Dump.Goal.FRONTLOADER_INDEX: FrontloaderDump,
                          Dump.Goal.GROUND_DRUM_INDEX: GroundDrumDump})
        self.back_selected = False
        self.selection_sub = self.create_subscription(
            ArmControlMode, 'arm_control_mode', self.on_selection, 10)

    def on_selection(self, message):
        # Preserve rear priority, including the both-selected teleop default.
        self.back_selected = message.back_arm_control == 1

    def make_routine(self, index, parameters):
        return self.variants[index](parameters, self.back_selected)


def main(args=None):
    run(DumpNode, args)


if __name__ == '__main__':
    main()
