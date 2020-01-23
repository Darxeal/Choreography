import math
from typing import List

from rlbot.utils.rendering.rendering_manager import RenderingManager
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, StepResult, PerDroneStep, StateSettingStep, \
    ParallelStep

from rlutilities.linear_algebra import vec3, rotation, dot, vec2, look_at, normalize, xy, angle_between, \
    axis_to_rotation, norm, clip, cross
from rlutilities.simulation import Ball, Input, Curve

from choreography.utils import BezierPath, direction, distance


class YeetTheBallOutOfTheUniverse(StateSettingStep):
    def set_ball_state(self, ball: Ball):
        ball.position = vec3(0, 0, 3000)
        ball.velocity = vec3(0, 0, 0)
        ball.angular_velocity = vec3(0, 0, 0)


BLUE_DRAGON_PATH = BezierPath([
    vec3(1060.62, 5000, 597.84),
    vec3(1060.62, 4535.96, 597.84),
    vec3(760.90, 3940.71, 597.84),
    vec3(25.88, 3451.79, 682.14),
    vec3(-80.05, 2844.85, 1011.74),
    vec3(658.04, 2427.09, 778.38),
    vec3(1537.42, 2140.13, 276.30),
    vec3(1600, 1500, 200),
    vec3(0, 0, 500),
    vec3(-1000, -1000, 500),
    vec3(0, -2000, 500),
    vec3(1000, -3000, 500),
    vec3(0, -4000, 500),
    vec3(-1000, -5000, 500),
])

PURPLE_DRAGON_PATH = BezierPath([
    vec3(-199, 5000, 510),
    vec3(-199, 4711, 590),
    vec3(-245, 4198, 601),
    vec3(-140, 3943, 604),
    vec3(221, 3634, 672),
    vec3(777, 3112, 891),
    vec3(745, 2883, 1164),
    vec3(575, 2635, 1360),
    vec3(267, 2456, 1389),
    vec3(-200, 2097, 1139),
    vec3(-490, 1460, 830),
    vec3(-470, 600, 700),
    vec3(950, -1000, 600),
    vec3(-1000, -3000, 600),
    vec3(0, -4000, 600),
    vec3(1380, -3880, 600),
    vec3(2760, -3760, 600),
])


class Dragon(StateSettingStep):
    duration = 10.0
    delay = 300
    curve: Curve = None

    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            p = self.time_since_start / self.duration * self.curve.length - self.delay * (
                    drone.id - self.target_indexes[0])
            p = self.curve.length - p

            dir = direction(drone.position, self.curve.point_at(p - 10))

            drone.position = self.curve.point_at(p)
            drone.velocity = dir * 500
            drone.angular_velocity = vec3(0, 0, 0)

            drone.orientation = look_at(dir, vec3(0, 0, 1))
            angle = dot(xy(drone.left()), xy(self.curve.tangent_at(p - 100))) * -5
            drone.orientation = dot(axis_to_rotation(drone.forward() * angle), drone.orientation)

    def render(self, renderer: RenderingManager):
        renderer.draw_polyline_3d(self.curve.points[::5], renderer.white())


class BlueDragon(Dragon):
    curve = Curve(BLUE_DRAGON_PATH.to_points(1000))
    target_indexes = range(0, 5)


class PurpleDragon(Dragon):
    curve = Curve(PURPLE_DRAGON_PATH.to_points(1000))
    target_indexes = range(5, 10)


class Boost(BlindBehaviorStep):

    def set_controls(self, controls: Input):
        controls.boost = True


class DragonsChoreography(Choreography):
    map_name = "ForbiddenTemple"

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        return ['blue_dragon.cfg'] * 5 + ['purple_dragon.cfg'] * 5

    @staticmethod
    def get_num_bots():
        return 10

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            ParallelStep([
                BlueDragon(),
                PurpleDragon(),
                Boost()
            ]),
        ]
