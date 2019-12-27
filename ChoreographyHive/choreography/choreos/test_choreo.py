import math
from typing import List

from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, StepResult, PerDroneStep, StateSettingStep

from rlutilities.linear_algebra import vec3, rotation, dot, vec2, look_at
from rlutilities.simulation import Ball, Input


class YeetTheBallOutOfTheUniverse(StateSettingStep):
    def set_ball_state(self, ball: Ball):
        ball.position = vec3(0, 0, 3000)
        ball.velocity = vec3(0, 0, 0)
        ball.angular_velocity = vec3(0, 0, 0)


class FormACircle(StateSettingStep):
    radius = 1200
    center = vec3(0, 0, 0)

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            angle = i * math.pi * 2 / len(drones)
            rot = rotation(angle)
            v = vec3(dot(rot, vec2(1, 0)))
            drone.position = v * self.radius + self.center
            drone.orientation = look_at(v * (-1)**i, vec3(0, 0, 1))
            drone.velocity = vec3(0, 0, 0)


class DriveBackward(BlindBehaviorStep):
    duration = 1.0

    def set_controls(self, controls: Input):
        controls.throttle = -0.4


class DriveForward(BlindBehaviorStep):
    duration = 1.0

    def set_controls(self, controls: Input):
        controls.throttle = 0.5

class JumpUpABit(BlindBehaviorStep):
    duration = 0.6

    def set_controls(self, controls: Input):
        controls.jump = True
        controls.pitch = 0.2
        controls.boost = True


class Test(Choreography):

    @staticmethod
    def get_num_bots():
        return 32

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            FormACircle(),
            DriveBackward(),
            JumpUpABit(),
            DriveForward(),
            DriveBackward(),
            JumpUpABit(),
            DriveForward(),
            DriveBackward(),
            JumpUpABit(),
            DriveForward(),
            DriveBackward()
        ]
