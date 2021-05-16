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
        appearances = ['rings.cfg'] * 64
        return appearances

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        teams = [0] * 32 + [1] * 32
        return teams

    @staticmethod
    def get_names(num_bots: int) -> List[str]:
        names = [str(i) for i in range(num_bots)]
        return names

    def generate_sequence(self):
        self.sequence = [
            Wait(1.0),
            RingSetup(),
            Rings(),
            HoldBoost(5.0)
        ]


CENTRE = vec3(-4000, 50_000, 4_000)
END = vec3(500, 44_500, 5_000)
ROTATION = 2.0
ANGLE = 0.1
TILT = 0.2
DIST = 800
RADIUS = 1000


def circle(angle):
    return vec3(cos(angle), sin(angle), 0)


class RingSetup(TwoTickStateSetStep):
    def set_drone_states(self, drones: List[Drone]):

        eyes = look_at(vec3(1, 0, 0), vec3(0, 0, 1))
        rot = dot(axis_to_rotation(vec3(0, 0, 1) * ROTATION), eyes)

        ring1_o = dot(axis_to_rotation(vec3(0, 1, 0) * ANGLE), eyes)
        ring1_o = dot(axis_to_rotation(vec3(1, 0, 0) * TILT), ring1_o)
        ring1_c = CENTRE + dot(rot, vec3(DIST/2, 0, 0))

        ring2_o = dot(axis_to_rotation(vec3(0, -1, 0) * ANGLE), eyes)
        ring2_o = dot(axis_to_rotation(vec3(-1, 0, 0) * TILT), ring2_o)
        ring2_c = CENTRE + dot(rot, vec3(-DIST/2, 0, 0))

        for index, drone in enumerate(drones):
            if index < 32:
                centre = ring1_c
                pos = ring1_c + dot(ring1_o, RADIUS * circle((index)/32 * 2 * pi))
            else:
                centre = ring2_c
                pos = ring2_c + dot(ring2_o, RADIUS * circle((index-32)/32 * 2 * pi))

            drone.orientation = look_at(vec3(0, 0, 1), normalize(pos - centre))
            drone.position = pos
            drone.velocity = vec3(0, 0, 0)


class Rings(DroneListStep):
    duration = 20.0

    def step(self, packet: GameTickPacket, drones: List[Drone]):
        motion_dir = (END - CENTRE) / self.duration

        eyes = look_at(vec3(1, 0, 0), vec3(0, 0, 1))
        rot = dot(axis_to_rotation(vec3(0, 0, 1) * ROTATION), eyes)
        center = CENTRE + motion_dir * self.time_since_start

        ring1_o = dot(axis_to_rotation(vec3(0, 0, 1) * self.time_since_start * -0.6), eyes)
        ring1_o = dot(axis_to_rotation(vec3(0, 1, 0) * ANGLE), ring1_o)
        ring1_o = dot(axis_to_rotation(vec3(1, 0, 0) * TILT), ring1_o)
        ring1_c = center + dot(rot, vec3(DIST/2, 0, 0))

        ring2_o = dot(axis_to_rotation(vec3(0, 0, 1) * self.time_since_start * -0.6), eyes)
        ring2_o = dot(axis_to_rotation(vec3(0, -1, 0) * ANGLE), ring2_o)
        ring2_o = dot(axis_to_rotation(vec3(-1, 0, 0) * TILT), ring2_o)
        ring2_c = center + dot(rot, vec3(-DIST/2, 0, 0))

        for index, drone in enumerate(drones):
            if index < 32:
                drone.hover.target = ring1_c + dot(ring1_o, RADIUS * circle((index)/32 * 2 * pi))
                drone.hover.up = normalize(drone.position - ring1_c)
            else:
                drone.hover.target = ring2_c + dot(ring2_o, RADIUS * circle((index-32)/32 * 2 * pi))
                drone.hover.up = normalize(drone.position - ring2_c)

            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls

class HoldBoost(PerDroneStep):
    def __init__(self, duration=1.0):
        super().__init__()
        self.duration = duration
    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        drone.controls.boost = True
