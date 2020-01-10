import math
from typing import List

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, StepResult, PerDroneStep, StateSettingStep

from rlutilities.linear_algebra import vec3, rotation, dot, vec2, look_at, mat3, norm, cross
from rlutilities.simulation import Ball, Input

from choreography.img_to_shape import convert_img_to_shape


class YeetTheBallOutOfTheUniverse(StateSettingStep):
    def set_ball_state(self, ball: Ball):
        ball.position = vec3(0, 0, 3000)
        ball.velocity = vec3(0, 0, 0)
        ball.angular_velocity = vec3(0, 0, 0)


class FormATriangle(StateSettingStep):
    center = vec3(0, -4000, 0)

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            angle = math.pi / 2
            rot = rotation(angle)
            v = vec3(dot(rot, vec2(1, 0)))
            v[2] = 0.25
            if i % 2 == 0:
                drone.position = self.center + vec3(math.floor(i / 2) * 250, math.floor(i / 2) * -250, 1750)
            else:
                drone.position = self.center + vec3((1 + math.floor(i / 2)) * -250, (1 + math.floor(i / 2)) * -250,
                                                    1750)
            drone.orientation = look_at(v, vec3(0, 0, 1))
            drone.velocity = vec3(0, 2300, 500)
            drone.angular_velocity = vec3(0, 0, 0)


class BoostUntilFast(PerDroneStep):
    duration = 1.5

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        if index == 0:
            pass
        elif index % 2 == 0:
            drone.aerial_turn.target = look_at(vec3(0, 1, 0.25), vec3(1, 0, 1))
            drone.aerial_turn.step(self.dt)
        else:
            drone.aerial_turn.target = look_at(vec3(0, 1, 0.25), vec3(-1, 0, 1))
            drone.aerial_turn.step(self.dt)
        drone.controls = drone.aerial_turn.controls
        drone.controls.boost = 1
        drone.controls.throttle = 1


class FlyOut(PerDroneStep):
    duration = 0.25

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        if index == 0:
            pass
        else:
            drone.controls.pitch = 1
        drone.controls.boost = 1
        drone.controls.throttle = 1


class Boost(PerDroneStep):
    duration = 1

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        drone.controls.boost = 1
        drone.controls.throttle = 1


class AirShow(Choreography):

    @staticmethod
    def get_num_bots():
        return 9

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            FormATriangle(),
            BoostUntilFast(),
            FlyOut(),
            Boost()
        ]
