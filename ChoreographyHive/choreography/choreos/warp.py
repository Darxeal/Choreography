from math import sin, cos, pi, sqrt
from typing import List

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, PerDroneStep, StateSettingStep, TwoTickStateSetStep

from rlutilities.linear_algebra import axis_to_rotation, cross, dot, look_at, norm, vec2, vec3, normalize

from .examples import Wait

class CheckerFlag(Choreography):
    map_name = "UtopiaRetro"

    @staticmethod
    def get_num_bots():
        return 64

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        appearances = ["flag.cfg"] * 64
        return appearances

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        line = [0, 1] * 4
        teams = (line + line[::-1]) * 4
        return teams

    @staticmethod
    def get_names(num_bots: int) -> List[str]:
        names = [str(i) for i in range(num_bots)]
        return names

    def generate_sequence(self):
        self.sequence = [
            Wait(1.0),
            FlagSetup(),
            Flag(),
        ]


CENTRE = vec3(0, 0, 5000)
DIRECTION = vec3(1, 0, 0)
SPACING = 200
ROW_SIZE = 8

class FlagSetup(TwoTickStateSetStep):
    def set_drone_states(self, drones: List[Drone]):
        hor = normalize(DIRECTION)
        ver = vec3(0, 0, 1)
        for index, drone in enumerate(drones):
            x = index % ROW_SIZE
            y = index // ROW_SIZE
            drone.position = CENTRE + (hor * x + ver * y) * SPACING
            drone.orientation = look_at(vec3(0, 0, 1), cross(hor, vec3(0, 0, 1)))
            drone.velocity = vec3(0, 0, 0)
            drone.angular_velocity = vec3(0, 0, 0)

class Flag(DroneListStep):
    duration = 20.0
    def step(self, packet: GameTickPacket, drones: List[Drone]):
        hor = normalize(DIRECTION)
        ver = vec3(0, 0, 1)
        perp = cross(hor, ver)

        start = CENTRE + hor * -200 * self.time_since_start

        for index, drone in enumerate(drones):
            x = index % ROW_SIZE
            y = index // ROW_SIZE
            target = start + (hor * x + ver * y) * SPACING
            target += perp * sin(2 * (self.time_since_start - (x + y)/6)) * 300
            drone.hover.target = target
            drone.hover.up = dot(perp, axis_to_rotation(vec3(0, 0, 1) * max(3 * self.time_since_start - (x + y)/3, 1E-9)))
            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls


class Explosion(Choreography):
    map_name = "AquaDome"

    @staticmethod
    def get_num_bots():
        return 64

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        appearances = ["bumblebee-appearance.cfg"] * num_bots
        return appearances

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        return [1] * num_bots

    @staticmethod
    def get_names(num_bots: int) -> List[str]:
        names = [str(i) for i in range(num_bots)]
        return names

    def generate_sequence(self):
        self.sequence = [
            ExplosionPrep(vec3(-10000, 0, 10000), 0),
            Boom(),
            ExplosionPrep(vec3(-10000, 0, 10000), 1),
            Boom(),
            ExplosionPrep(vec3(-10000, 0, 10000), 2),
            Boom(),
            ExplosionPrep(vec3(-10000, 0, 10000), 3),
            Boom(),
            ExplosionPrep(vec3(-10000, 0, 10000), 4),
            Boom(),
            ExplosionPrep(vec3(-10000, 0, 10000), 5),
            Boom(),
            ExplosionPrep(vec3(-10000, 0, 10000), 6),
            Boom(),
            ExplosionPrep(vec3(-10000, 0, 10000), 20),
            Boom(),
        ]


class ExplosionPrep(TwoTickStateSetStep):
    radius = 1000
    def __init__(self, position: vec3, freq: float = 20):
        super().__init__()
        self.position = position
        self.freq = freq

    def set_drone_states(self, drones: List[Drone]):
        num_bots = len(drones)

        for index, drone in enumerate(drones):
            t = 2 * ((index + 0.5) / num_bots) - 1.0
            x = sqrt(1 - t**2) * cos(t * self.freq)
            y = sqrt(1 - t**2) * sin(t * self.freq)
            z = t

            dir = vec3(x, y, z)
            drone.position = self.position + dir * self.radius
            drone.orientation = look_at(dir, vec3(0, 0, 1))
            drone.velocity = dir * 2300
            drone.angular_velocity = vec3(10, 20, 30)

from random import random

class Boom(BlindBehaviorStep):
    duration = 0.5
    def set_controls(self, controls):
        controls.boost = True
        # controls.yaw = 2 * random() - 1
        # controls.pitch = 2 * random() - 1
        # controls.roll = 2 * random() - 1

