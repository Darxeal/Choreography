import math
from typing import List

from rlbot.utils.structures.game_interface import GameInterface
from rlutilities.linear_algebra import vec3, rotation, dot, vec2, look_at, cross
from rlutilities.simulation import Ball, Input, Curve

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, StateSettingStep, ParallelStep
from choreography.paths.AirShowPath import get_paths
from choreography.utils.vector_math import direction


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
            print(i, drone.position)


class Fly(StateSettingStep):
    duration = 20
    distance_between_body_parts = 300
    curve: Curve = None

    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            t = self.time_since_start / self.duration * self.curve.length
            t -= self.distance_between_body_parts * (drone.id - self.target_indexes[0])
            t = self.curve.length - t

            pos = self.curve.point_at(t)
            pos_ahead = self.curve.point_at(t - 500)
            pos_behind = self.curve.point_at(t + 30)

            facing_direction = direction(pos_behind, pos)
            target_left = cross(facing_direction, direction(pos, pos_ahead))
            target_up = cross(target_left, facing_direction)
            up = drone.up() + target_up * 0.9 + vec3(0, 0, 0.1)
            target_orientation = look_at(facing_direction, up)

            drone.position = pos
            drone.velocity = facing_direction * (self.curve.length / self.duration)
            drone.angular_velocity = vec3(0, 0, 0)
            drone.orientation = target_orientation


class bot0(Fly):
    curve = Curve(get_paths()[0].to_points(1000))
    target_indexes = range(0, 1)


class bot1(Fly):
    curve = Curve(get_paths()[1].to_points(1000))
    target_indexes = range(1, 2)


class bot2(Fly):
    curve = Curve(get_paths()[2].to_points(1000))
    target_indexes = range(2, 3)


class bot3(Fly):
    curve = Curve(get_paths()[3].to_points(1000))
    target_indexes = range(3, 4)


class bot4(Fly):
    curve = Curve(get_paths()[4].to_points(1000))
    target_indexes = range(4, 5)


class bot5(Fly):
    curve = Curve(get_paths()[5].to_points(1000))
    target_indexes = range(5, 6)


class bot6(Fly):
    curve = Curve(get_paths()[6].to_points(1000))
    target_indexes = range(6, 7)


class bot7(Fly):
    curve = Curve(get_paths()[7].to_points(1000))
    target_indexes = range(7, 8)


class bot8(Fly):
    curve = Curve(get_paths()[8].to_points(1000))
    target_indexes = range(8, 9)


class Boost(BlindBehaviorStep):
    target_indexes = range(0, 9)

    def set_controls(self, controls: Input):
        controls.boost = True


class AirShow(Choreography):

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        appearances = ['default.cfg'] * num_bots
        appearances[0] = 'AirShowWhite.cfg'
        appearances[1] = 'AirShowWhite.cfg'
        appearances[2] = 'AirShowWhite.cfg'
        appearances[3] = 'AirShowBlue.cfg'
        appearances[4] = 'AirShowRed.cfg'
        appearances[5] = 'AirShowBlue.cfg'
        appearances[6] = 'AirShowRed.cfg'
        appearances[7] = 'AirShowBlue.cfg'
        appearances[8] = 'AirShowRed.cfg'
        return appearances

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        # Every other bot is on the orange team.
        teams = [0] * num_bots
        teams[0] = 1
        teams[1] = 1
        teams[2] = 1
        teams[3] = 0
        teams[4] = 1
        teams[5] = 0
        teams[6] = 1
        teams[7] = 0
        teams[8] = 1
        return teams

    @staticmethod
    def get_num_bots():
        return 9

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            FormATriangle(),
            ParallelStep([
                bot0(),
                bot1(),
                bot2(),
                bot3(),
                bot4(),
                bot5(),
                bot6(),
                bot7(),
                bot8(),
                Boost()
            ]),
        ]
