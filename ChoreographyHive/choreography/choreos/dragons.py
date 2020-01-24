from math import pi
from typing import List

from rlbot.utils.rendering.rendering_manager import RenderingManager
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface
from rlutilities.linear_algebra import vec3, dot, look_at, xy, axis_to_rotation, rotation_to_axis, cross, sgn
from rlutilities.simulation import Ball, Input, Curve

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, StateSettingStep, ParallelStep, PerDroneStep
from choreography.utils import BezierPath, direction


class Setup(StateSettingStep):
    target_indexes = range(10, 40)

    def set_ball_state(self, ball: Ball):
        ball.position = vec3(0, 0, -500)
        ball.velocity = vec3(0, 0, 0)
        ball.angular_velocity = vec3(0, 0, 0)

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            shifted_index = i - 15
            sign = 1 if shifted_index >= 0 else -1
            if sign > 0:
                shifted_index = 15 - shifted_index
            drone.position = vec3(shifted_index * 200 + sign * 800, -600 * sign, 20)
            drone.velocity = vec3(0, 0, 0)
            drone.angular_velocity = vec3(0, 0, 0)
            drone.orientation = look_at(vec3(0, 1, 0) * sign, vec3(0, 0, 1))


BLUE_DRAGON_PATH = BezierPath([
    vec3(1060, 5110, 487),
    vec3(1060, 4657, 597),
    vec3(760, 3940, 597),
    vec3(25, 3451, 682),
    vec3(-80, 2844, 1011),
    vec3(658, 2427, 778),
    vec3(1537, 2140, 276),
    vec3(1280, 1440, 200),
    vec3(-220, 1170, 440),
    vec3(-1140, 420, 210),
    vec3(-780, -330, 190),
    vec3(740, -690, 270),
    vec3(1200, -1100, 770),
    vec3(425, -1360, 1270),
    vec3(-270, -1230, 830),
    vec3(-1060, -1060, 530),
    vec3(-1490, -1020, 960),
    vec3(-980, -970, 1370),
    vec3(-510, -920, 890),
    vec3(-1180, -940, 490),
    vec3(-2400, -1180, 650),
    vec3(-2800, -2010, 810),
    vec3(-2250, -2400, 970),
    vec3(-1880, -1500, 1130),
    vec3(-2350, -120, 1450),
    vec3(-2255, 1117, 1427),
    vec3(-1880, 1475, 1425),
    vec3(-1430, 2130, 1660),
    vec3(-1600, 2480, 1157),
    vec3(-945, 1815, 1046),
    vec3(230, 2190, 1415),
    vec3(1630, 1500, 1360),
    vec3(2100, 570, 1420),
    vec3(1850, -1170, 1450),
    vec3(30, -2180, 1400),
    vec3(-1920, -1000, 1420),
    vec3(-1810, 1200, 1440),
    vec3(-60, 2190, 1400),
    vec3(1920, 1030, 1410),
    vec3(1810, -1180, 1420),
    vec3(240, -2170, 1430),
    vec3(-1330, -1720, 1400),
    vec3(-2230, 400, 1370),
    vec3(-1140, 1910, 1340),
])

PURPLE_DRAGON_PATH = BezierPath([
    vec3(-199, 5000, 560),
    vec3(-199, 4711, 590),
    vec3(-245, 4198, 601),
    vec3(-140, 3943, 614),
    vec3(747, 3462, 811),
    vec3(745, 2883, 1164),
    vec3(575, 2635, 1360),
    vec3(267, 2456, 1389),
    vec3(-200, 2097, 1139),
    vec3(-620, 1550, 490),
    vec3(-470, 600, 200),
    vec3(1000, 60, 400),
    vec3(1430, 850, 600),
    vec3(330, 1290, 800),
    vec3(-60, 70, 1000),
    vec3(590, -990, 540),
    vec3(1240, -2050, 1190),
    vec3(580, -2750, 1490),
    vec3(-80, -2520, 810),
    vec3(-840, -1550, 790),
    vec3(-930, -580, 950),
    vec3(740, -560, 480),
    vec3(1630, -1660, 220),
    vec3(2470, -1880, 790),
    vec3(2350, -1220, 1490),
    vec3(2150, -100, 1410),
    vec3(2105, 1290, 1515),
    vec3(2240, 1820, 1620),
    vec3(2770, 1570, 1370),
    vec3(2210, 850, 1380),
    vec3(2160, -340, 1420),
    vec3(1690, -1170, 1350),
    vec3(1290, -1880, 1450),
    vec3(340, -2140, 1430),
    vec3(-1030, -2020, 1410),
    vec3(-2010, -870, 1390),
    vec3(-2240, 160, 1370),
    vec3(-1920, 1190, 1430),
    vec3(-1260, 1810, 1430),
    vec3(160, 2180, 1430),
    vec3(1810, 1270, 1420),
    vec3(2100, -670, 1410),
    vec3(-110, -2200, 1400),
    vec3(-2230, -410, 1390),
    vec3(-1320, 1760, 1385),
    vec3(790, 2030, 1380),
    vec3(2250, -110, 1410),
    vec3(210, -2040, 1440),
])


class Dragon(StateSettingStep):
    duration = 40.0
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

    # def render(self, renderer: RenderingManager):
    #     renderer.draw_polyline_3d(self.curve.points[::5], renderer.white())


class BlueDragon(Dragon):
    curve = Curve(BLUE_DRAGON_PATH.to_points(1000))
    target_indexes = range(0, 5)


class PurpleDragon(Dragon):
    curve = Curve(PURPLE_DRAGON_PATH.to_points(1000))
    target_indexes = range(5, 10)


class RingOfFire(PerDroneStep):
    ring_radius = 500
    rotation_radius = 2200
    height = 1400
    starting_rotation: float = None
    rotation_start_delay = 20

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        rotation = self.starting_rotation + max(0, self.time_since_start - self.rotation_start_delay) / 2
        v = dot(axis_to_rotation(vec3(0, 0, 1) * rotation), vec3(1, 0, 0))
        center = vec3(0, 0, self.height) + v * self.rotation_radius
        facing = cross(v, vec3(0, 0, 1))

        n = len(self.target_indexes)
        i = index - self.target_indexes[0]
        angle = i / n * pi * 2
        pos = center + dot(vec3(0, 0, 1), axis_to_rotation(facing * angle)) * self.ring_radius

        if pos[2] > self.height + self.ring_radius - self.time_since_start * 200:
            drone.hover.target = pos
            drone.hover.up = facing
            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls
            drone.controls.jump = True


class Ring1(RingOfFire):
    target_indexes = range(10, 25)
    starting_rotation = pi


class Ring2(RingOfFire):
    target_indexes = range(25, 40)
    starting_rotation = 0


class Boost(BlindBehaviorStep):
    target_indexes = range(0, 10)

    def set_controls(self, controls: Input):
        controls.boost = True


class DragonsChoreography(Choreography):
    map_name = "Mannfield_Night"

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        return ['blue_dragon.cfg'] * 5 + ['purple_dragon.cfg'] * 5 + ['fire.cfg'] * 30

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        return [0] * 10 + [1] * 30

    @staticmethod
    def get_num_bots():
        return 40

    def generate_sequence(self):
        self.sequence = [
            Setup(),
            ParallelStep([
                BlueDragon(),
                PurpleDragon(),
                Boost(),
                Ring1(),
                Ring2()
            ]),
        ]
