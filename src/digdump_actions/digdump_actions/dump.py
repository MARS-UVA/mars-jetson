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


class DumpNode(RoutineServer):
    def __init__(self):
        super().__init__('dump', Dump, RobotState.DUMP, DEFAULTS,
                         {Dump.Goal.BRUNO_INDEX: BrunoDump})
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
