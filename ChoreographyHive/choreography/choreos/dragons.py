from typing import List

from rlbot.utils.rendering.rendering_manager import RenderingManager
from rlbot.utils.structures.game_interface import GameInterface
from rlutilities.linear_algebra import vec3, dot, look_at, xy, axis_to_rotation, rotation_to_axis, cross
from rlutilities.simulation import Ball, Input, Curve

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, StateSettingStep, ParallelStep
from choreography.utils import BezierPath, direction


class YeetTheBallOutOfTheUniverse(StateSettingStep):
    def set_ball_state(self, ball: Ball):
        ball.position = vec3(0, 0, 3000)
        ball.velocity = vec3(0, 0, 0)
        ball.angular_velocity = vec3(0, 0, 0)


BLUE_DRAGON_PATH = BezierPath([
    vec3(1060, 5110, 597),
    vec3(1060, 4657, 597),
    vec3(760, 3940, 597),
    vec3(25, 3451, 682),
    vec3(-80, 2844, 1011),
    vec3(658, 2427, 778),
    vec3(1537, 2140, 276),
    vec3(1600, 1500, 200),
    vec3(1040, 750, 350),
    vec3(-1000, -1000, 500),
    vec3(1000, -3000, 320),
    vec3(0, -4000, 500),
    vec3(-1470, -3760, 500),
    vec3(-1630, -3000, 590),
    vec3(-1600, -2090, 510),
    vec3(-1590, -1900, 870),
    vec3(-1580, -2300, 1190),
    vec3(-1570, -2700, 770),
    vec3(-1620, -2380, 500),
    vec3(-1580, -960, 640),
    vec3(-1540, 460, 780),
    vec3(-1500, 1880, 920),
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
    duration = 15.0
    delay = 300
    curve: Curve = None

    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            t = self.time_since_start / self.duration * self.curve.length
            t -= self.delay * (drone.id - self.target_indexes[0])
            t = self.curve.length - t

            pos = self.curve.point_at(t)
            pos_ahead = self.curve.point_at(t - 500)
            pos_behind = self.curve.point_at(t + 100)

            facing_direction = direction(pos_behind, pos)
            target_left = cross(facing_direction, direction(pos, pos_ahead))
            target_up = cross(target_left, facing_direction)
            up = drone.up() + target_up * 1.5  # + vec3(0, 0, 0.01)
            target_orientation = look_at(facing_direction, up)

            drone.position = pos
            drone.velocity = facing_direction * (self.curve.length / self.duration)
            drone.angular_velocity = vec3(0, 0, 0)
            drone.orientation = target_orientation

    # def render(self, renderer: RenderingManager):
    #     renderer.draw_polyline_3d(self.curve.points[::5], renderer.white())


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
