import math
from typing import List
from dataclasses import dataclass

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface
from rlbot.agents.base_agent import SimpleControllerState

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
            drone.orientation = look_at(vec3(2,0,3), vec3(1,0,0))
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
class Drawing(StateSettingStep):

    def __init__(self, image, origin=vec3(0,0,18), duration=2.0):
        super().__init__()
        self.duration = duration
        self.origin = origin
        self.shape = convert_img_to_shape(image)
    
    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            if i < len(self.shape):
                drone.position = self.origin + self.shape[i]
                drone.orientation = mat3(1,0,0,0,1,0,0,0,1)
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
        [0,        16],
        [1,2,      17,18,       32,33],
        [3,4,5,    19,20,21,    34,35,36],
        [6,7,8,9,  22,23,24,25, 37,38,39,40],
        [10,11,12, 26,27,28,    41,42,43],
        [13,14,    29,30,       44,45],
        [15,       31]
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
                        radius = 2000 - diff * ((self.time_since_start - self.radius_shrink_start) / self.radius_shrink_duration)
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
        [0,        16],
        [1,2,      17,18,       32,33],
        [3,4,5,    19,20,21,    34,35,36],
        [6,7,8,9,  22,23,24,25, 37,38,39,40],
        [10,11,12, 26,27,28,    41,42,43],
        [13,14,    29,30,       44,45],
        [15,       31]
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
            angle = (-1)**i * -math.pi / 2
            x = -self.x_distance * (-1)**i
            y = (self.y_distance + self.gap_offset * (i//2)) * (-1)**i
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

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            Grid(),
            BaseGraph(),
            Parabola(),
            BaseGraph(),
            CosSin(),
            BaseGraph(),
            Wave()
        ]

class Grid(TwoTickStateSetStep):
    spacing = 200

    def set_drone_states(self, drones: List[Drone]):
        s = int( math.sqrt( len(drones) ) ) # Side length
        for i, drone in enumerate(drones):
            # Get grid pos.
            x = (i //s) - (s-1)/2
            y = (i % s) - (s-1)/2
            drone.position = vec3(x * self.spacing, y * self.spacing, 800) # 800 is base height
            drone.orientation = euler_to_rotation(vec3(math.pi/2, 0, 0))
            drone.velocity = vec3(0, 0, 100)
            drone.angular_velocity = vec3(0, 0, 0)

class BaseGraph(DroneListStep):
    duration = math.pi
    rotation_speed = 0
    spacing = 200

    def func(self, x, y):
        return 0

    def step(self, packet: GameTickPacket, drones: List[Drone]):
        s = int( math.sqrt( len(drones) ) ) # Side length
        for i, drone in enumerate(drones):
            # Get grid pos.
            x = (i //s) - (s-1)/2
            y = (i % s) - (s-1)/2
            # Get height from func.
            z = 800 + self.func(x, y) # 800 is base height

            drone.hover.target = vec3(x * self.spacing, y * self.spacing, z)
            rot = rotation(self.rotation_speed * self.time_since_start * 2)
            drone.hover.up = vec3(dot(rot, vec2(1, 0)))
            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls

class Parabola(BaseGraph):
    
    def func(self, x, y):
        return 40*(x**2 + y**2) - 200

class CosSin(BaseGraph):

    def func(self, x, y):
        return 250*(math.cos(x) + math.sin(y))

class Wave(BaseGraph):
    duration = 4*math.pi

    def func(self, x, y):
        t = self.time_since_start
        return 150*(math.sin(x/2+t))

# SNAKE
class Clones(Choreography):
    @staticmethod
    def get_num_bots():
        return 10

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        a = SimpleControllerState()
        a.throttle = True

        b = SimpleControllerState()
        b.jump = True
        b.pitch = 1.0

        movements = [
            MovementInInterval(0.0, 3.0, a),
            MovementInInterval(3.0, 4.0, b)
        ]
        GoForwardAndThenDoAJumpOrSomething = HardcodedMovement(movements)

        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            StackThemUp(),
            GoForwardAndThenDoAJumpOrSomething
        ]

class StackThemUp(StateSettingStep):
    pos = vec3(0, -2000, 20)
    height = 100

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            drone.position = self.pos
            drone.position[2] += i * self.height
            drone.orientation = euler_to_rotation(vec3(0, math.pi/2, 0))
            drone.velocity = vec3(0, 0, 0)
            drone.angular_velocity = vec3(0, 0, 0)

@dataclass
class MovementInInterval:
    start : float
    end : float
    controls : SimpleControllerState

# Pass in a list of MovementInIntervals and it automatically completes the moves with each drone.
# If you have the temptation to use clone_delay = 0, use BlindBehaviourStep instead.
class HardcodedMovement(PerDroneStep):

    def __init__(self, movements : List[MovementInInterval], clone_delay : float = 1.0):
        self.movements = movements
        self.clone_delay = clone_delay
        super().__init__()

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        delay = index * self.clone_delay
        for movement in self.movements:
            if movement.start + delay < self.time_since_start < movement.end + delay:
                # Convert SimpleControllerState to Input
                drone.controls.throttle = movement.controls.throttle
                drone.controls.steer = movement.controls.steer
                drone.controls.pitch = movement.controls.pitch
                drone.controls.yaw = movement.controls.yaw
                drone.controls.roll = movement.controls.roll
                drone.controls.jump = movement.controls.jump
                drone.controls.boost = movement.controls.boost
                drone.controls.handbrake = movement.controls.handbrake

        if index == packet.num_cars - 1:
            self.finished = self.time_since_start > delay + self.movements[-1].end