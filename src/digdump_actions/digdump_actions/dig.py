"""Bruno's feedback-driven dig sequence and its action server."""

import math

from autonomy_msgs.action import Dig
from robot_control_msgs.msg import RobotState
from serial_msgs.msg import Position

from digdump_actions.server import Command, RoutineServer, run


DEFAULTS = {
    'actuator_speed_aerial': 1.0,
    'actuator_speed_ground': 0.75,
    'dig_speed': 0.75,
    'dig_time': 3.0,
    'actuator_extend_length_aerial': 0.69,
    'actuator_extend_length_ground': 0.9,
    'actuator_raise_target': 0.6,
}


class BrunoDig:
    """Lower each arm, dig for a fixed time, then raise each arm."""

    def __init__(self, parameters):
        self.p = parameters
        self.phase = 'lowering'
        self.started = None

    def step(self, now, positions):
        if positions is None:
            return self.phase, Command(), False
        front, back = positions
        p = self.p
        if self.phase == 'lowering':
            def lower_speed(position):
                if position >= p['actuator_extend_length_ground']:
                    return 0.0
                if position >= p['actuator_extend_length_aerial']:
                    return -p['actuator_speed_ground']
                return -p['actuator_speed_aerial']

            if min(front, back) < p['actuator_extend_length_ground']:
                return self.phase, Command(
                    front_arm=lower_speed(front), back_arm=lower_speed(back),
                    front_drum=-p['dig_speed'], back_drum=-p['dig_speed']), False
            self.phase = 'digging'
            self.started = None
            return self.phase, Command(), False

        if self.phase == 'digging':
            if self.started is None:
                self.started = now
            if now - self.started < p['dig_time']:
                return self.phase, Command(
                    front_drum=p['dig_speed'], back_drum=p['dig_speed']), False
            self.phase = 'raising'
            return self.phase, Command(), False

        target = p['actuator_raise_target']
        return self.phase, Command(
            front_arm=p['actuator_speed_aerial'] if front > target else 0.0,
            back_arm=p['actuator_speed_aerial'] if back > target else 0.0,
        ), max(front, back) <= target
        
class FrontloaderDig:
    """Time-based placeholder for lowering, driving, and raising the loader."""

    LOWER_SPEED = 0.5
    RAISE_SPEED = 0.5
    DRIVE_SPEED = 0.25
    LOWER_TIME = 2.0
    DRIVE_TIME = 12.0
    RAISE_TIME = 2.0

    def __init__(self, parameters):
        self.phase = 'lowering'
        self.started = None

    def step(self, now, positions=None):
        if self.started is None:
            self.started = now

        if self.phase == 'lowering':
            if now - self.started < self.LOWER_TIME:
                return self.phase, Command(front_arm=-self.LOWER_SPEED), False
            self.phase = 'driving'
            self.started = now
            return self.phase, Command(), False

        if self.phase == 'driving':
            if now - self.started < self.DRIVE_TIME:
                return self.phase, Command(drive=self.DRIVE_SPEED), False
            self.phase = 'raising'
            self.started = now
            return self.phase, Command(), False

        if now - self.started < self.RAISE_TIME:
            return self.phase, Command(front_arm=self.RAISE_SPEED), False
        return self.phase, Command(), True

class GroundDrumDig:
    """Spin while lowering, dig while reversing, then stop and raise."""

    LOWER_SPEED_AERIAL = 1.0
    LOWER_SPEED_GROUND = 0.75
    LOWER_SLOWDOWN_TARGET = 0.69
    RAISE_SPEED = 1.0
    DIG_SPEED = 0.75
    DIG_TIME = 30.0
    DRIVE_SPEED = 0.25
    LOWER_TARGET = 0.9
    RAISE_TARGET = 0.6

    def __init__(self, _parameters):
        self.phase = 'lowering'
        self.started = None

    def step(self, now, positions):
        if positions is None:
            return self.phase, Command(), False

        front, back = positions
        if self.phase == 'lowering':
            def lower_speed(position):
                if position >= self.LOWER_TARGET:
                    return 0.0
                if position >= self.LOWER_SLOWDOWN_TARGET:
                    return -self.LOWER_SPEED_GROUND
                return -self.LOWER_SPEED_AERIAL

            if min(front, back) < self.LOWER_TARGET:
                return self.phase, Command(
                    front_arm=lower_speed(front),
                    back_arm=lower_speed(back),
                    front_drum=self.DIG_SPEED,
                    back_drum=self.DIG_SPEED,
                ), False
            self.phase = 'digging'
            self.started = now

        if self.phase == 'digging':
            if now - self.started < self.DIG_TIME:
                return self.phase, Command(
                    front_drum=self.DIG_SPEED,
                    back_drum=self.DIG_SPEED,
                    drive=-self.DRIVE_SPEED,
                ), False
            self.phase = 'raising'
            return self.phase, Command(), False

        return self.phase, Command(
            front_arm=self.RAISE_SPEED if front > self.RAISE_TARGET else 0.0,
            back_arm=self.RAISE_SPEED if back > self.RAISE_TARGET else 0.0,
        ), max(front, back) <= self.RAISE_TARGET


class DigNode(RoutineServer):
    def __init__(self):
        super().__init__('dig', Dig, RobotState.DIG, DEFAULTS,
                         {Dig.Goal.BRUNO_INDEX: BrunoDig,
                          Dig.Goal.FRONTLOADER_INDEX: FrontloaderDig,
                          Dig.Goal.GROUND_DRUM_INDEX: GroundDrumDig},
                         )
        self.actuator_positions = None
        self.position_sub = self.create_subscription(Position, 'position', self.on_position, 10)

    def on_position(self, message):
        values = (message.front_actuator_position, message.back_actuator_position)
        self.actuator_positions = values if all(math.isfinite(v) for v in values) else None

    def positions(self):
        return self.actuator_positions


def main(args=None):
    run(DigNode, args)


if __name__ == '__main__':
    main()
