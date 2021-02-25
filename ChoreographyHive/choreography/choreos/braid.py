from math import sin, cos, pi, sqrt
from typing import List

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, PerDroneStep, StateSettingStep, TwoTickStateSetStep

from rlutilities.linear_algebra import look_at, vec3, normalize

from .examples import Wait

class Braids(Choreography):
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
            BraidSetup(),
            Braid(),
            Wait(3.0),
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
    duration = 20.0
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
