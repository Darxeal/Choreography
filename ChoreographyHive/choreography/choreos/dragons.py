from math import pi
from typing import List

from rlbot.utils.rendering.rendering_manager import RenderingManager
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface
from rlutilities.linear_algebra import vec3, dot, look_at, xy, axis_to_rotation, rotation_to_axis, cross, sgn
from rlutilities.simulation import Ball, Input, Curve

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, StateSettingStep, ParallelStep, PerDroneStep, TwoTickStateSetStep
from choreography.utils import BezierPath, direction


class RingsSetup(StateSettingStep):
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


class HideDragons(StateSettingStep):
    target_indexes = range(0, 10)

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            drone.position = vec3(i * 100, 6000, 0)


BLUE_DRAGON_PATH = BezierPath([
    vec3(1060, 5110, 567),
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
    vec3(-2380, -1430, 580),
    vec3(-2560, -2115, 630),
    vec3(-2040, -2400, 680),
    vec3(-1665, -1860, 905),
    vec3(-2232, -1460, 1017),
    vec3(-2741, -1945, 1045),
    vec3(-2271, -2430, 1073),
    vec3(-1720, -1690, 1130),
    vec3(-2035, -905, 1290),
    vec3(-2350, -120, 1450),
    vec3(-2215, 827, 1427),
    vec3(-1880, 1475, 1425),
    vec3(-945, 1815, 1246),
    vec3(-507, 2222, 1230),
    vec3(-498, 2841, 1222),
    vec3(-887, 3239, 1233),
    vec3(-1356, 2838, 1195),
    vec3(-1324, 2255, 1288),
    vec3(-110, 2270, 1355),
    vec3(622, 2010, 1473),
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
    vec3(-2220, -420, 1385),
    vec3(-2230, 400, 1370),
    vec3(-1880, 760, 1340),
    vec3(-1150, 170, 1310),
    vec3(-410, -1080, 1330),
    vec3(400, 80, 1400),
    vec3(-320, 670, 1470),
    vec3(-900, -110, 1540),
    vec3(-760, -1140, 1610),
    vec3(-1840, -1000, 1530),
    vec3(-2030, 145, 1527),
    vec3(-1850, 990, 1370),
    vec3(-1380, 1575, 1395),
    vec3(-950, 2077, 1392),
    vec3(-350, 2028, 1436),
    vec3(310, 2200, 1300),
    vec3(1810, 1490, 1390),
    vec3(2255, 660, 1420),
    vec3(2360, -60, 1450),
    vec3(2050, -1065, 1410),
    vec3(1640, -1690, 1310),
    vec3(1055, -1750, 1310),
    vec3(587, -2010, 1300),
    vec3(180, -1970, 1440),
    vec3(-630, -1985, 1505),
    vec3(-1180, -1927, 1477),
    vec3(-1990, -1340, 1310),
    vec3(-2370, 70, 1410),
    vec3(-1210, 730, 1070),
    vec3(-370, 700, 890),
    vec3(310, 380, 630),
    vec3(480, -10, 540),
    vec3(70, -350, 530),
    vec3(-220, 30, 690),
    vec3(110, 280, 940),
    vec3(280, -60, 1210),
    vec3(90, -230, 1457),
    vec3(-30, -20, 1705),
    vec3(180, -40, 2130),
    vec3(-130, 20, 3010),
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
    vec3(2000, 940, 1395),
    vec3(2090, -110, 1410),
    vec3(1430, -20, 1440),
    vec3(200, 970, 1340),
    vec3(-1050, 10, 1410),
    vec3(-490, -1090, 1480),
    vec3(230, -480, 1550),
    vec3(-250, 840, 1470),
    vec3(-1170, 290, 1390),
    vec3(-1620, 650, 1430),
    vec3(-1710, 1650, 1380),
    vec3(-1460, 1850, 1540),
    vec3(-1173, 1786, 1581),
    vec3(-887, 1722, 1462),
    vec3(-606, 1893, 1203),
    vec3(-335, 2255, 1205),
    vec3(2, 2342, 1477),
    vec3(865, 1900, 1415),
    vec3(1700, 1370, 1420),
    vec3(2015, 670, 1425),
    vec3(2150, -30, 1430),
    vec3(1550, -1470, 1310),
    vec3(1095, -1955, 1475),
    vec3(630, -1910, 1400),
    vec3(130, -2150, 1515),
    vec3(-300, -2050, 1410),
    vec3(-740, -2005, 1380),
    vec3(-1290, -1710, 1350),
    vec3(-1780, -1070, 1440),
    vec3(-2100, -570, 1490),
    vec3(-2240, 190, 1440),
    vec3(-1280, 1070, 1220),
    vec3(-120, 1550, 1120),
    vec3(550, 500, 800),
    vec3(400, -130, 700),
    vec3(-20, -300, 890),
    vec3(-200, -47, 1235),
    vec3(-40, 115, 1440),
    vec3(130, -40, 1660),
    vec3(-20, -140, 2150),
    vec3(20, -70, 2550),
])

TOTAL_SPEED = 0.7


class Dragon(StateSettingStep):
    duration = 65.0 / TOTAL_SPEED
    distance_between_body_parts = 300
    curve: Curve = None

    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            t = self.time_since_start / self.duration * self.curve.length
            t -= self.distance_between_body_parts * (drone.id - self.target_indexes[0])

            if t < 0:
                continue

            t = self.curve.length - t

            pos = self.curve.point_at(t)
            pos_ahead = self.curve.point_at(t - 500)
            pos_behind = self.curve.point_at(t + 300)

            facing_direction = direction(pos_behind, pos)
            target_left = cross(facing_direction, direction(pos, pos_ahead))
            target_up = cross(target_left, facing_direction)
            up = drone.up() + target_up * 0.9 + vec3(0, 0, 0.1)
            target_orientation = look_at(facing_direction, up)

            drone.position = pos_behind
            drone.velocity = facing_direction * (self.curve.length / self.duration)
            drone.angular_velocity = vec3(0, 0, 0)
            drone.orientation = target_orientation

    # def render(self, renderer: RenderingManager):
    #     renderer.draw_polyline_3d(self.curve.points[::5], renderer.white())


class BlueDragon(Dragon):
    curve = Curve(BLUE_DRAGON_PATH.to_points(2000))
    target_indexes = range(0, 5)


class PurpleDragon(Dragon):
    curve = Curve(PURPLE_DRAGON_PATH.to_points(2000))
    target_indexes = range(5, 10)


class RingOfFire(PerDroneStep):
    ring_radius = 500
    rotation_radius = 2200
    height = 1400
    starting_rotation: float = None
    rotation_start_delay = 20 / TOTAL_SPEED
    rotation_speed = 0.45 * TOTAL_SPEED

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        rotation = self.starting_rotation + max(0,
                                                self.time_since_start - self.rotation_start_delay) * self.rotation_speed
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


class Wait(BlindBehaviorStep):
    duration = 5.0

    def set_controls(self, controls: Input):
        pass


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
            Wait(),
            HideDragons(),
            RingsSetup(),
            ParallelStep([
                BlueDragon(),
                PurpleDragon(),
                Boost(),
                Ring1(),
                Ring2()
            ]),
        ]
