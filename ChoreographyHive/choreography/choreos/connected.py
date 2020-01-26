import math
from typing import List

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, StepResult, PerDroneStep, \
    StateSettingStep, TwoTickStateSetStep

from rlutilities.linear_algebra import vec2, vec3, euler_to_rotation, rotation, dot, normalize, norm
from rlutilities.simulation import Ball, Input

from .examples import YeetTheBallOutOfTheUniverse, Wait

class ConnectedChoreo(Choreography):
    
    @staticmethod
    def get_num_bots():
        return 64

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            FourStacks(),
            ResetAttr(),
            Wait(2.0),
            ForwardThenQuadHelix()
        ]

class FourStacks(TwoTickStateSetStep):
    height = 50
    radius = 3000
    offset = 400

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            if i in range(0, 16):
                pos = vec3(-self.radius, -self.offset, 20)
                angle = 0

            elif i in range(16, 32):
                pos = vec3(self.offset, -self.radius, 20)
                angle = math.pi / 2

            elif i in range(32, 48):
                pos = vec3(self.radius, self.offset, 20)
                angle = math.pi

            elif i in range(48, 64):
                pos = vec3(-self.offset, self.radius, 20)
                angle = 3 * math.pi / 2

            drone.position = pos
            drone.position[2] += (i % 16) * self.height
            drone.orientation = euler_to_rotation(vec3(0, angle, 0))
            drone.velocity = vec3(0, 0, 0)
            drone.angular_velocity = vec3(0, 0, 0)

class ResetAttr(PerDroneStep):

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        self.finished = True
        drone.since_jumped = None

class ForwardThenQuadHelix(PerDroneStep):
    delay = 0.8
    duration = 60.0

    def step(self, packet: GameTickPacket, drone: Drone, index: int):

        if drone.since_jumped is None:
            # Control throttle start and jump.
            if self.time_since_start > self.delay * (index % 16):
                # Speed controller.
                drone.controls.throttle = 1.0 if max(abs(drone.velocity[0]), abs(drone.velocity[1])) < 300 else 0.03

                # If near half-line
                if norm(vec3(0,0,0) - drone.position) < 600:
                    drone.since_jumped = 0.0
                    drone.controls.jump = True

        # Create helix after jump.
        else:
            # Increment timer.
            drone.since_jumped += self.dt

            # rise
            if drone.since_jumped < 16:
                height = 200 + drone.since_jumped * 80 # speed of rise
            # fall
            elif drone.since_jumped < 24:
                height = 1480 - (drone.since_jumped - 16) * 80
            # second rise
            elif drone.since_jumped < 32:
                height = 840 + (drone.since_jumped - 24) * 80
            else:
                height = 800 + 200 * (index // 16)

            if drone.since_jumped < 16:
                radius = 550 + drone.since_jumped**3 / 10
            elif drone.since_jumped < 32:
                radius = 960 - (drone.since_jumped - 16) * 20
            else:
                radius = 640 + 200 * (index // 16)

            if self.time_since_start < 55:
                angle = drone.since_jumped * 0.35 # rotation speed
                # Offset angle.
                angle += (index // 16) * (math.pi / 2)
            else:
                angle = (2 * math.pi / 16) * (index % 16)

            # Set hover target and controller.
            rot = rotation(angle)
            v = vec3(dot(rot, vec2(1, 0)))
            drone.hover.target = v * radius
            drone.hover.target[2] = height
            drone.hover.up = normalize(drone.position)
            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls

            if drone.since_jumped < 0.2:
                drone.controls.jump = True
                drone.controls.boost = False