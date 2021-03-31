from math import sin, cos, pi, sqrt

from rlbot.utils.game_state_util import GameState
from rlutilities.mechanics import AerialTurn
from typing import List

from rlbot.utils.structures.game_data_struct import FieldInfoPacket, GameTickPacket
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
        return 63

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        appearances = ["flag.cfg"] * num_bots
        return appearances

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        line = [0, 1] * 10 + [0]
        invert = [0 if x else 1 for x in line]
        return line + invert + line

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


CENTRE = vec3(-16500, 29000, 5750)
DIRECTION = vec3(0, 1, 0)
H_SPACING = 175
V_SPACING = 175
ROW_SIZE = 21

class FlagSetup(TwoTickStateSetStep):
    def set_drone_states(self, drones: List[Drone]):
        hor = normalize(DIRECTION)
        ver = vec3(0, 0, 1)
        for index, drone in enumerate(drones):
            x = index % ROW_SIZE
            y = index // ROW_SIZE
            drone.position = CENTRE + (hor * x * H_SPACING) + (ver * y * V_SPACING)
            drone.orientation = look_at(vec3(0, 0, 1), cross(hor, vec3(0, 0, 1)))
            drone.velocity = vec3(0, 0, 0)
            drone.angular_velocity = vec3(0, 0, 0)

    def set_ball_state(self, ball):
        ball.position = vec3(7400, 30150, -1000) # vec3(7400, 30150, 6600)
        ball.velocity = vec3(0, 0, 0)

class Flag(DroneListStep):
    duration = 60.0
    def step(self, packet: GameTickPacket, drones: List[Drone]):
        hor = normalize(DIRECTION)
        ver = vec3(0, 0, 1)
        perp = cross(hor, ver)

        start = CENTRE # + hor * -250 * self.time_since_start

        for index, drone in enumerate(drones):
            x = index % ROW_SIZE
            y = index // ROW_SIZE
            target = start + (hor * x * H_SPACING) + (ver * y * V_SPACING)
            target += perp * sin(2 * (self.time_since_start - (x + y)/6)) * 700
            drone.hover.target = target
            drone.hover.up = dot(perp, axis_to_rotation(vec3(0, 0, 1) * max(3 * self.time_since_start - (x + y)/3, 1E-9)))
            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls


class Explosion(Choreography):
    map_name = "UtopiaRetro"
    # map_name = "AquaDome"

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
            # ExplosionPrep(vec3(10_000, 0, 5_000), 21),
            # Boom(vec3(10_000, 0, 5_000)),
            Test(),
            Wait(1),
        ]

class Test(StateSettingStep):
    def perform(self, *args):
        interface = next((arg for arg in args if isinstance(arg, GameInterface)), None)
        if interface:
            field_info = FieldInfoPacket()
            interface.update_field_info_packet(field_info)
            print([g.location for g in field_info.goals[:field_info.num_goals]])
        return super().perform(*args)


class ExplosionPrep(TwoTickStateSetStep):
    duration = 1.0
    radius = 200
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

class Boom(PerDroneStep):
    duration = 3.0

    def __init__(self, position: vec3, freq: float = 20):
        super().__init__()
        self.position = position

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        dir = normalize(drone.position - self.position)
        drone.aerial_turn.target = look_at(dir, vec3(0, 0, 1))
        drone.aerial_turn.step(self.dt)
        drone.controls = drone.aerial_turn.controls
        # drone.controls.yaw = 1.0
        # drone.controls.pitch = 1.0
        # drone.controls.roll = -1.0
        drone.controls.boost = True
        
