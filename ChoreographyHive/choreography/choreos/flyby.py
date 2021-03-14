from math import sin, cos, pi, sqrt
from typing import List
from random import random
from rlbot.utils.game_state_util import GameInfoState, GameState

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, PerDroneStep, StateSettingStep, StepResult, TwoTickStateSetStep

from rlutilities.linear_algebra import axis_to_rotation, dot, look_at, mat3, norm, vec3, normalize

from .examples import Wait


class Flyby(Choreography):
    map_name = "UtopiaRetro"

    @staticmethod
    def get_num_bots():
        return 7

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        appearances = ['bumblebee-appearance.cfg'] * num_bots
        return appearances

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        teams = [1] * num_bots
        return teams

    @staticmethod
    def get_names(num_bots: int) -> List[str]:
        names = [str(i) for i in range(num_bots)]
        return names

    def generate_sequence(self):
        self.sequence = [
            SetGrav(),
            Wait(1.0),
            FlyBySetup(),
            TheFlyBy(),
        ]


class SetGrav(StateSettingStep):
    grav = -300
    def perform(self, *args) -> StepResult:
        interface = next((arg for arg in args if isinstance(arg, GameInterface)), None)
        if interface:
            interface.set_game_state(GameState(game_info=GameInfoState(world_gravity_z=self.grav)))
        return super().perform(*args)


START = vec3(-20000, -40000, 10000)


class FlyBySetup(StateSettingStep):
    radius = 500
    def set_drone_states(self, drones: List[Drone]):
        drones[0].position = START
        drones[0].velocity = vec3(1000, 0, 0)
        drones[0].orientation = mat3(1, 0, 0, 0, 1, 0, 0, 0, 1)
        for i, drone in enumerate(drones[1:]):
            angle = i * 2 * pi / 6
            pos = vec3(0, cos(angle) * self.radius, sin(angle) * self.radius)
            drone.position = START + pos
            drone.velocity = vec3(1000, 0, 0)
            drone.orientation = mat3(1, 0, 0, 0, 1, 0, 0, 0, 1)


class TheFlyBy(PerDroneStep):
    duration = 10.0

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        target_direction = normalize(vec3(4, 0, 1))
        o = look_at(target_direction, vec3(0, 0, 1))
        drone.aerial_turn.target = dot(axis_to_rotation(vec3(1, 0, 0) * self.time_since_start / 5), o)
        drone.aerial_turn.step(self.dt)
        drone.controls = drone.aerial_turn.controls
        drone.controls.boost = True
