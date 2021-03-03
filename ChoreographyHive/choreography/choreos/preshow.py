from math import sin, cos, pi, sqrt
from typing import List
from random import random

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, PerDroneStep, StateSettingStep, TwoTickStateSetStep

from rlutilities.linear_algebra import axis_to_rotation, dot, look_at, norm, vec3, normalize

from .examples import Wait

class Preshow(Choreography):
    map_name = "UtopiaRetro"

    @staticmethod
    def get_num_bots():
        return 63

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        appearances = ['braid1.cfg', 'braid1.cfg', 'braid2.cfg'] * (num_bots // 3)
        return appearances

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        teams = [0, 1, 1] * (num_bots // 3)
        return teams

    @staticmethod
    def get_names(num_bots: int) -> List[str]:
        names = [str(i) for i in range(num_bots)]
        return names

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            # BraidSetup(),
            # Braid(),
            # Wait(1.5),
            # Transition(),
            DiscSetup(),
            Disc()
        ]


CENTRE = vec3(-15000, -15000, -500)

def braid(t):
    t %= 2 * pi
    if 3 * pi / 2 > t > pi / 2:
        t = 2 * pi - t
    return sqrt(1 + cos(2 * t)) * vec3(cos(t), sin(t), 0)


class BraidSetup(TwoTickStateSetStep):
    radius = 600
    height = -100

    def set_drone_states(self, drones: List[Drone]):
        for index, drone in enumerate(drones):
            i = index % 3
            theta = i / 3 * 2 * pi

            drone.position = CENTRE + braid(theta) * self.radius
            drone.position[2] += index * self.height 
            drone.orientation = look_at(vec3(0, 0, 1), normalize(drone.position - CENTRE))
            drone.velocity = vec3(0, 0, 0)
            drone.angular_velocity = vec3(0, 0, 700)


class Braid(PerDroneStep):
    duration = 15.0
    radius = 600
    height = -100

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        i = index % 3

        theta = i / 3 * 2 * pi
        t_diff = max(0, self.time_since_start - (index / 3) * 0.3)
        theta += t_diff

        target = CENTRE + braid(theta) * self.radius
        target[2] += index * self.height + 800 * self.time_since_start

        drone.hover.up = normalize(drone.position - CENTRE)
        drone.hover.target = target
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls


def disc(i):
    i += 1
    return (i / 64)**0.5 * vec3(cos(i), sin(i), 0)


class Transition(PerDroneStep):
    duration = 20.0
    radius = 2000
    ori = look_at(vec3(1, 0, 0), vec3(0, 0, 1))

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        pos = dot(self.ori, self.radius * disc(index))
        target = CENTRE + pos
        target[2] += 8000

        if norm(drone.position - target) > 600:
            target = drone.position + 500 * normalize(target - drone.position)

        drone.hover.up = normalize(drone.position - CENTRE)
        drone.hover.target = target
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls


class DiscSetup(TwoTickStateSetStep):
    radius = 2000
    ori = look_at(vec3(1, 0, 0), vec3(0, 0, 1))

    def set_drone_states(self, drones: List[Drone]):     
        for index, drone in enumerate(drones):

            pos = dot(self.ori, self.radius * disc(index))
            drone.position = CENTRE + pos
            drone.position[2] += 8000
            drone.orientation = look_at(vec3(0, 0, 1), normalize(drone.position - CENTRE))
            drone.velocity = vec3(0, 0, 0)
            drone.angular_velocity = vec3(0, 0, 0)


class Disc(DroneListStep):
    duration = 60.0
    radius = 2000
    start_pos = CENTRE + vec3(0, 0, 8000)

    def step(self, packet: GameTickPacket, drones: List[Drone]):
        ori = look_at(vec3(1, 0, 0), vec3(0, 0, 1))
        ori = dot(axis_to_rotation(vec3(1, 0, 0) * self.time_since_start / 7), ori)
        ori = dot(axis_to_rotation(vec3(0, 1, 0) * self.time_since_start / 5), ori)
        ori = dot(axis_to_rotation(vec3(0, 0, 1) * self.time_since_start / 3), ori)
        ori = dot(axis_to_rotation(vec3(1, 0, 0) * self.time_since_start / 8), ori)

        centre = self.start_pos + vec3(0, -500, 0) * self.time_since_start

        for index, drone in enumerate(drones):
            pos = dot(ori, self.radius * disc(index))
            target = centre + pos
            drone.hover.up = normalize(drone.position - CENTRE)
            drone.hover.target = target
            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls

