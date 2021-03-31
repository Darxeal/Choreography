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
        return 13

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
            Wait(10.0),
            FlyBySetup(),
            TheFlyBy(),
        ]


class SetGrav(StateSettingStep):
    grav = -1
    def perform(self, *args) -> StepResult:
        interface = next((arg for arg in args if isinstance(arg, GameInterface)), None)
        if interface:
            interface.set_game_state(GameState(game_info=GameInfoState(world_gravity_z=self.grav)))
        return super().perform(*args)


START = vec3(-25000, -41000, 5000)


class FlyBySetup(StateSettingStep):
    radius = 700
    def set_drone_states(self, drones: List[Drone]):
        drones[0].position = START
        drones[0].velocity = vec3(0, 2300, 0)
        drones[0].orientation = look_at(vec3(0, 1, 0), vec3(0, 0, 1))
        for i, drone in enumerate(drones[1:]):
            angle = i * 2 * pi / 12
            pos = vec3(cos(angle) * self.radius, 0, sin(angle) * self.radius)
            drone.position = START + pos
            drone.velocity = vec3(0, 2300, 0)
            drone.orientation = look_at(normalize(vec3(0, 1000, 0) - pos), normalize(pos))


class TheFlyBy(PerDroneStep):
    radius = 700
    duration = 10.0

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        if index == 0:
            pos = vec3(0, 0, 0)
        else:
            angle = (index + 1) * 2 * pi / 12
            pos = vec3(cos(angle) * self.radius, 0, sin(angle) * self.radius)

        o = look_at(normalize(vec3(0, 2000, 0) - pos), normalize(pos))

        drone.aerial_turn.target = dot(axis_to_rotation(vec3(0, 3, 0) * self.time_since_start), o)
        drone.aerial_turn.step(self.dt)
        drone.controls = drone.aerial_turn.controls
        drone.controls.boost = True
