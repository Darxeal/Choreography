from math import sin, cos, pi, sqrt
from typing import List

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, PerDroneStep, StateSettingStep, TwoTickStateSetStep

from rlutilities.linear_algebra import axis_to_rotation, dot, look_at, norm, vec3, normalize

from .examples import Wait

class LookUp(Choreography):
    map_name = "UtopiaRetro"

    @staticmethod
    def get_num_bots():
        return 64

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        appearances = ['braid1.cfg'] * 32 + ['braid2.cfg'] * 32
        return appearances

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        teams = [1] * 64
        return teams

    @staticmethod
    def get_names(num_bots: int) -> List[str]:
        names = [str(i) for i in range(num_bots)]
        return names

    def generate_sequence(self):
        self.sequence = [
            Wait(1.0),
            RingSetup(),
            Rings()
        ]


CENTRE = vec3(0, 0, 5000)
ANGLE = 0.1
TILT = 0.2
DIST = 800
RADIUS = 1000


def circle(angle):
    return vec3(cos(angle), sin(angle), 0)


class RingSetup(TwoTickStateSetStep):
    def set_drone_states(self, drones: List[Drone]):
        for index, drone in enumerate(drones):
            o = look_at(vec3(1, 0, 0), vec3(0, 0, 1))
            if index < 32:
                o = dot(axis_to_rotation(vec3(0, 1, 0) * ANGLE), o)
                o = dot(axis_to_rotation(vec3(1, 0, 0) * TILT), o)
                centre = CENTRE + vec3(DIST/2, 0, 0)
            else:
                o = dot(axis_to_rotation(vec3(0, -1, 0) * ANGLE), o)
                o = dot(axis_to_rotation(vec3(-1, 0, 0) * TILT), o)
                centre = CENTRE + vec3(-DIST/2, 0, 0)

            pos = centre + dot(o, RADIUS * circle((index%32)/32 * 2 * pi))
            drone.orientation = look_at(vec3(0, 0, 1), normalize(pos - centre))
            drone.position = pos
            drone.velocity = vec3(0, 0, 0)


class Rings(DroneListStep):
    duration = 30.0

    def step(self, packet: GameTickPacket, drones: List[Drone]):
        for index, drone in enumerate(drones):
            o = look_at(vec3(1, 0, 0), vec3(0, 0, 1))
            centre = CENTRE + vec3(0, 300, 0) * self.time_since_start
            if index < 32:
                o = dot(axis_to_rotation(vec3(0, 0, 1) * self.time_since_start * -0.6), o)
                o = dot(axis_to_rotation(vec3(0, 1, 0) * ANGLE), o)
                o = dot(axis_to_rotation(vec3(1, 0, 0) * TILT), o)
                centre += vec3(DIST/2, 0, 0)
            else:
                o = dot(axis_to_rotation(vec3(0, 0, -1) * self.time_since_start * 0.6), o)
                o = dot(axis_to_rotation(vec3(0, -1, 0) * ANGLE), o)
                o = dot(axis_to_rotation(vec3(-1, 0, 0) * TILT), o)
                centre += vec3(-DIST/2, 0, 0)

            drone.hover.up = normalize(centre - drone.position)
            drone.hover.target = centre + dot(o, RADIUS * circle((index%32)/32 * 2 * pi))
            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls
