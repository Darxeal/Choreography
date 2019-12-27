import math
from typing import List
from typing import List

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, StepResult, PerDroneStep, StateSettingStep

from rlutilities.linear_algebra import vec3, rotation, dot, vec2, look_at
from rlutilities.simulation import Ball, Input
from rlutilities.mechanics import Boostdash, Dodge


class YeetTheBallOutOfTheUniverse(StateSettingStep):
    def set_ball_state(self, ball: Ball):
        ball.position = vec3(0, 0, 3000)
        ball.velocity = vec3(0, 0, 0)
        ball.angular_velocity = vec3(0, 0, 0)


class FormACircle(StateSettingStep):
    radius = 2000
    center = vec3(0, 0, 0)

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            angle = i * math.pi * 2 / len(drones)
            rot = rotation(angle)
            v = vec3(dot(rot, vec2(1, 0)))
            drone.position = v * self.radius + self.center
            drone.orientation = look_at(v * -1, vec3(0, 0, 1))
            drone.velocity = vec3(0, 0, 0)


class WaveDash:
    duration: float = 5

    def __init__(self):
        self.start_time: float = 0
        assert self.duration > 0.0

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        finished = []
        for drone in drones:
            if drone.mechanic is None:
                drone.mechanic = Boostdash(drone)
            drone.mechanic.step(1 / 60)
            drone.controls = drone.mechanic.controls
            finished.append(drone.mechanic.finished)
        # return StepResult(finished=all(finished))
        time = packet.game_info.seconds_elapsed
        if not self.start_time:
            self.start_time = time
        return StepResult(finished=time > self.start_time + self.duration)


class JeroenChoreography(Choreography):

    @staticmethod
    def get_num_bots():
        return 32

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            FormACircle(),
            WaveDash()
        ]
