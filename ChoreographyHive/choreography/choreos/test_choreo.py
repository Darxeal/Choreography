import cmath
import math
from typing import List
from dataclasses import dataclass

import numpy
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, StepResult, PerDroneStep, \
    StateSettingStep, TwoTickStateSetStep

from rlutilities.linear_algebra import vec3, rotation, dot, vec2, look_at, mat3, norm, normalize, \
    xy, axis_to_rotation, euler_to_rotation
from rlutilities.simulation import Ball, Input

from choreography.img_to_shape import convert_img_to_shape

from .examples import YeetTheBallOutOfTheUniverse, FormACircle, Wait, FlyUp


# HEX FLIP COOL CLIP
class HexDoubleFlip(Choreography):

    @staticmethod
    def get_num_bots():
        return 6

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            HexSetup(),
            BoostUntilFast(),
            BackflipBoostyThing()
        ]


class HexSetup(StateSettingStep):
    radius = 300
    center = vec3(-2000, 0, 100)

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            angle = i * math.pi * 2 / len(drones)
            rot = rotation(angle)
            v = vec3(dot(rot, vec2(1, 0)))
            drone.position = v * self.radius + self.center
            drone.orientation = look_at(vec3(2, 0, 3), vec3(1, 0, 0))
            drone.velocity = vec3(0, 0, 500)
            drone.angular_velocity = vec3(0, 0, 0)


class BoostUntilFast(DroneListStep):
    def step(self, packet: GameTickPacket, drones: List[Drone]):
        self.finished = norm(drones[0].velocity) > 1000

        for drone in drones:
            drone.controls.pitch = 0
            drone.controls.boost = True


class BackflipBoostyThing(BlindBehaviorStep):
    duration = 6.0

    def set_controls(self, controls: Input):
        controls.pitch = 0.5
        controls.boost = True


# AUTOMATIC STATE SETTING INTO DRAWING
class Dickbutt(Choreography):
    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            Drawing('dickbutt.png', origin=vec3(-1000, 1500, 18)),
            Wait(1.0)
        ]

class Drawing(TwoTickStateSetStep):

    def __init__(self, image, origin=vec3(0, 0, 18)):
        super().__init__()
        self.origin = origin
        self.shape = convert_img_to_shape(image)

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            if i < len(self.shape):
                drone.position = self.origin + self.shape[i]
                drone.orientation = mat3(1, 0, 0, 0, 1, 0, 0, 0, 1)
                drone.velocity = vec3(0, 0, 0)
            else:
                drone.position = vec3(0, 0, 3000)


# CIRCLES AND SPHERE FORMATION TESTS
class CirclesAndSpheres(Choreography):

    @staticmethod
    def get_num_bots():
        return 45

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            FormACircle(),
            Wait(1.0),
            FlyUp(),
            HoverSpinUp(),
            HoverSpinDown(),
            SphereFormation(),
            HoverOrbit()
        ]


class HoverSpinUp(PerDroneStep):
    duration = 6.0

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        drone.hover.up = normalize(drone.position)
        clockwise_rotation = axis_to_rotation(vec3(0, 0, self.time_since_start / 4))
        position_on_circle = normalize(xy(drone.position)) * (2000 - self.time_since_start * 200)
        drone.hover.target = dot(clockwise_rotation, position_on_circle)
        drone.hover.target[2] = 1000
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls


class HoverSpinDown(PerDroneStep):
    duration = 6.0

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        drone.hover.up = normalize(drone.position)
        clockwise_rotation = axis_to_rotation(vec3(0, 0, 1.5 - self.time_since_start / 4))
        position_on_circle = normalize(xy(drone.position)) * (800 + self.time_since_start * 200)
        drone.hover.target = dot(clockwise_rotation, position_on_circle)
        drone.hover.target[2] = 1000
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls


class SphereFormation(DroneListStep):
    duration = 12.0

    separation_duration = 3.0
    recirculation_start = 6.5
    radius_shrink_start = 3.0
    radius_shrink_duration = 6.0

    layers = [
        [0, 16],
        [1, 2, 17, 18, 32, 33],
        [3, 4, 5, 19, 20, 21, 34, 35, 36],
        [6, 7, 8, 9, 22, 23, 24, 25, 37, 38, 39, 40],
        [10, 11, 12, 26, 27, 28, 41, 42, 43],
        [13, 14, 29, 30, 44, 45],
        [15, 31]
    ]
    heights = [
        1500,
        1400,
        1250,
        1000,
        750,
        600,
        500,
    ]
    radii = [
        200,
        450,
        600,
        650,
        600,
        450,
        200,
    ]

    def step(self, packet: GameTickPacket, drones: List[Drone]):
        for drone in drones:
            drone.hover.up = normalize(drone.position)

            for i, layer in enumerate(self.layers):
                if drone.id in layer:
                    # Calculate radius
                    if self.time_since_start < self.radius_shrink_start:
                        radius = 2000

                    elif self.time_since_start < self.radius_shrink_start + self.radius_shrink_duration:
                        diff = 2000 - self.radii[i]
                        radius = 2000 - diff * (
                                    (self.time_since_start - self.radius_shrink_start) / self.radius_shrink_duration)
                    else:
                        radius = self.radii[i]

                    # Calculate xy position
                    if self.time_since_start > self.recirculation_start:
                        a = layer.index(drone.id)
                        angle = a * math.pi * 2 / len(layer)
                        rot = rotation(angle)
                        pos_xy = vec3(dot(rot, vec2(1, 0)))

                    else:
                        pos_xy = xy(drone.position)

                    # Combine xy and radius
                    drone.hover.target = normalize(pos_xy) * radius

                    # Get height
                    if self.time_since_start < self.separation_duration:
                        diff = 1000 - self.heights[i]
                        height = 1000 - diff * (self.time_since_start / self.separation_duration)

                    else:
                        height = self.heights[i]

                    drone.hover.target[2] = height
                    break

            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls


class HoverOrbit(PerDroneStep):
    duration = 8.0

    layers = [
        [0, 16],
        [1, 2, 17, 18, 32, 33],
        [3, 4, 5, 19, 20, 21, 34, 35, 36],
        [6, 7, 8, 9, 22, 23, 24, 25, 37, 38, 39, 40],
        [10, 11, 12, 26, 27, 28, 41, 42, 43],
        [13, 14, 29, 30, 44, 45],
        [15, 31]
    ]
    heights = [
        1500,
        1400,
        1250,
        1000,
        750,
        600,
        500,
    ]
    radii = [
        200,
        450,
        600,
        650,
        600,
        450,
        200,
    ]

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        for i, layer in enumerate(self.layers):
            if index in layer:
                drone.hover.up = normalize(drone.position)
                clockwise_rotation = axis_to_rotation(vec3(0, 0, 0.3))
                position_on_circle = normalize(xy(drone.position)) * self.radii[i]
                drone.hover.target = dot(clockwise_rotation, position_on_circle)
                drone.hover.target[2] = self.heights[i]
                break

        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls


# DOUBLE HELIX

class DoubleHelix(Choreography):

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        appearances = ['WillRedBlue.cfg'] * num_bots
        # appearances[0::4] = ['WillYellowGreen.cfg'] * round(num_bots / 4)
        # appearances[1::4] = ['WillYellowGreen.cfg'] * round(num_bots / 4)
        return appearances

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        # Every other bot is on the orange team.
        teams = [0] * num_bots
        teams[1::2] = [1] * round(num_bots / 2)
        return teams

    @staticmethod
    def get_num_bots():
        return 32

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            TwoLineSetup(),
            Wait(1.0),
            ForwardThenHelix()
        ]


class TwoLineSetup(StateSettingStep):
    y_distance = 500
    x_distance = 300
    gap_offset = 300

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            angle = (-1) ** i * -math.pi / 2
            x = -self.x_distance * (-1) ** i
            y = (self.y_distance + self.gap_offset * (i // 2)) * (-1) ** i
            drone.position = vec3(x, y, 20)
            drone.orientation = euler_to_rotation(vec3(0, angle, 0))
            drone.velocity = vec3(0, 0, 0)
            drone.angular_velocity = vec3(0, 0, 0)


class ForwardThenHelix(PerDroneStep):
    duration = 13.0
    radius = 500

    def step(self, packet: GameTickPacket, drone: Drone, index: int):

        if drone.position[2] < 25:
            drone.since_jumped = 0.0

            # Go forward
            drone.controls.throttle = 1.0 if abs(drone.velocity[1]) < 500 else 0.01

            # If near half-line
            if abs(drone.position[1]) < 200:
                drone.controls.jump = True

        else:
            drone.since_jumped += self.dt

            height = 50 + drone.since_jumped * 150
            angle = 1.0 + drone.since_jumped * 1.2
            if index % 2 == 0: angle += math.pi

            rot = rotation(angle)
            v = vec3(dot(rot, vec2(1, 0)))
            drone.hover.target = v * self.radius
            drone.hover.target[2] = height

            drone.hover.up = normalize(drone.position)
            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls


# F(X,Y) GRAPHER

class GraphTest(Choreography):

    @staticmethod
    def get_num_bots():
        return 64

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        return 64 * ['graph.cfg']

    # @staticmethod
    # def get_teams(num_bots: int) -> List[int]:
    #     teams = [0] * num_bots
    #     return teams

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            Grid(),
            BaseGraph(),
            Wave(),
            Water(),
            BaseGraph(),
            Saddle(),
            BaseGraph(),
            Pants(),
            # Parabola(),
            # CosSin(),
            # WindMill(),
            # YeetEquation(),
            # Limit(),
            # Jochem(),
            # LogarithmReal(),
        ]


class Grid(TwoTickStateSetStep):
    spacing = 200

    def set_drone_states(self, drones: List[Drone]):
        s = int(math.sqrt(len(drones)))  # Side length
        for i, drone in enumerate(drones):
            # Get grid pos.
            x = (i // s) - (s - 1) / 2
            y = (i % s) - (s - 1) / 2
            drone.position = vec3(x * self.spacing, y * self.spacing, 800)  # 800 is base height
            drone.orientation = euler_to_rotation(vec3(math.pi / 2, 0, 0))
            drone.velocity = vec3(0, 0, 100)
            drone.angular_velocity = vec3(0, 0, 0)


class BaseGraph(DroneListStep):
    duration = 2
    rotation_speed = 0
    spacing = 200

    def func(self, x, y):
        return 0

    def step(self, packet: GameTickPacket, drones: List[Drone]):
        s = int(math.sqrt(len(drones)))  # Side length
        for i, drone in enumerate(drones):
            # Get grid pos.
            x = (i // s) - (s - 1) / 2
            y = (i % s) - (s - 1) / 2
            # Get height from func.
            z = 800 + self.func(x, y)  # 800 is base height

            drone.hover.target = vec3(x * self.spacing, y * self.spacing, z)
            rot = rotation(self.rotation_speed * self.time_since_start * 2)
            drone.hover.up = vec3(dot(rot, vec2(1, 0)))
            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls


class Parabola(BaseGraph):

    def func(self, x, y):
        return 40 * (x ** 2 + y ** 2) - 200


class CosSin(BaseGraph):

    def func(self, x, y):
        return 250 * (math.cos(x) + math.sin(y))


class WindMill(BaseGraph):
    duration = 4 * math.pi

    def func(self, x, y):
        t = self.time_since_start
        return 1000 * (numpy.sign(x * y) * numpy.sign(1 - (x * 9) ** 2 + (y * 9) ** 2) / 9)


class Wave(BaseGraph):
    duration = 2 * math.pi

    def func(self, x, y):
        t = self.time_since_start
        return 150 * (math.sin(x / 2 + 2*t))

class YeetEquation(BaseGraph):
    duration = 5

    def func(self, x, y):
        t = self.time_since_start
        t_0 = 2
        c = 0.5
        a = 1/(4 * math.pi * c * (t+t_0))
        b = -(x**2 + y**2) / (4 * c * (t+t_0))
        return 20000 * a * math.exp(b)

class Water(BaseGraph):
    duration = 2 * math.pi

    def func(self, x, y):
        t = self.time_since_start
        return 250 * (math.sin(x / 2 + t)) * (math.cos(y / 2 + t))


class Saddle(BaseGraph):
    duration = 4 * math.pi

    def func(self, x, y):
        t = self.time_since_start
        return 4*x*y*t*math.cos(t)


class Jochem(BaseGraph):
    duration = 4

    def func(self, x, y):
        t = self.time_since_start
        return 300 * t * x/(x**2+y**2+0.3)


class Limit(BaseGraph):
    duration = 4 * math.pi

    def func(self, x, y):
        t = self.time_since_start
        return 10*t*math.cos(t)*x/(y + 0.001)


class Will(BaseGraph):
    duration = 5

    def func(self, x, y):
        t = self.time_since_start
        return 10 *(math.sin(1.5*t) - 0.5) * (x**2 + y**2)


class LogarithmReal(BaseGraph):
    duration = 4 * math.pi

    def func(self, x, y):
        t = self.time_since_start
        return 200 * math.cos(t) * (cmath.sqrt(x+y*1j)).real


class Pants(BaseGraph):
    duration = 4 * math.pi

    def func(self, x, y):
        t = self.time_since_start
        return 275 * math.sin(t) * (cmath.sqrt(x+y*1j)).imag

# HARDCODED CLONES

class Clones(Choreography):
    @staticmethod
    def get_num_bots():
        return 10

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            StackThemUp(),
            GoForwardAndThenDoAJumpOrSomething()
        ]


class StackThemUp(StateSettingStep):
    pos = vec3(0, -2000, 20)
    height = 50

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            drone.position = self.pos
            drone.position[2] += i * self.height
            drone.orientation = euler_to_rotation(vec3(0, math.pi / 2, 0))
            drone.velocity = vec3(0, 0, 0)
            drone.angular_velocity = vec3(0, 0, 0)


@dataclass
class MovementInInterval:
    start: float
    end: float
    controls: Input


# Pass in a list of MovementInIntervals and it automatically completes the moves with each drone.
# If you have the temptation to use clone_delay = 0, use BlindBehaviourStep instead.
class HardcodedMovement(PerDroneStep):

    def __init__(self, movements: List[MovementInInterval], clone_delay: float = 1.0):
        self.movements = movements
        self.clone_delay = clone_delay
        super().__init__()

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        delay = index * self.clone_delay
        for movement in self.movements:
            if movement.start + delay < self.time_since_start < movement.end + delay:
                drone.controls = movement.controls

        if index == packet.num_cars - 1:
            self.finished = self.time_since_start > delay + self.movements[-1].end


class GoForwardAndThenDoAJumpOrSomething(HardcodedMovement):

    def __init__(self):
        a = Input()
        a.throttle = True

        b = Input()
        b.jump = True
        b.pitch = 1.0

        movements = [
            MovementInInterval(0.0, 3.0, a),
            MovementInInterval(3.0, 4.2, b)
        ]
        super().__init__(movements, clone_delay=0.8)


# Unused cool sphere
class CoolSphere(PerDroneStep):
    duration = 30.0
    height = 1100
    radius = 850
    unwind_start_time = 10.0
    max_frequency = 30.0

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        if self.time_since_start > self.unwind_start_time:
            f = self.max_frequency - (self.time_since_start - self.unwind_start_time)
        else:
            f = self.max_frequency

        z = (index - 31.5) / 32 # For 64 bots :^)
        x = math.sqrt(1 - z**2) * math.cos(z * f)
        y = math.sqrt(1 - z**2) * math.sin(z * f)

        target = vec3(x, y, z) * self.radius
        target[2] += self.height

        drone.hover.up = normalize(drone.position)
        drone.hover.target = target
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls