import math
from typing import List

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, StepResult, PerDroneStep, \
    StateSettingStep, TwoTickStateSetStep

from rlutilities.linear_algebra import vec2, vec3, euler_to_rotation, rotation, dot, normalize
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
            ResetAttr(),
            TwinTowers(),
            Wait(2.0),
            ForwardThenHelix()
        ]

class PezDispensers(TwoTickStateSetStep):
    x_distance = -300
    y_distance = 2000
    height = 50

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            x = self.x_distance * (-1)**i
            y = self.y_distance * (-1)**i
            z = 20 + self.height * (i//2)
            angle =  (-math.pi / 2) * (-1)**i

            drone.position = vec3(x, y, z)
            drone.orientation = euler_to_rotation(vec3(0, angle, 0))
            drone.velocity = vec3(0, 0, 0)
            drone.angular_velocity = vec3(0, 0, 0)

class ResetAttr(PerDroneStep):

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        self.finished = True
        drone.since_jumped = None

class ForwardThenHelix(PerDroneStep):
    delay = 0.6
    duration = 25.0

    def step(self, packet: GameTickPacket, drone: Drone, index: int):

        if drone.since_jumped is None:
            # Control throttle start and jump.
            if self.time_since_start > self.delay * (index//2):
                # Go forward
                drone.controls.throttle = 1.0 if abs(drone.velocity[1]) < 500 else 0.03

                # If near half-line
                if abs(drone.position[1]) < 500:
                    drone.since_jumped = 0.0
                    drone.controls.jump = True

        else:
            # Increment timer.
            drone.since_jumped += self.dt

            # Create helix after jump.
            height = 50 + drone.since_jumped * 80 # speed of rise
            angle = 1.0 + drone.since_jumped * 0.5 # rotation speed
            radius = 450 * drone.since_jumped

            # Opposite side if index is even.
            if index % 2 == 0: angle += math.pi

            # Set hover target and controller.
            rot = rotation(angle)
            v = vec3(dot(rot, vec2(1, 0)))
            drone.hover.target = v * radius
            drone.hover.target[2] = height
            drone.hover.up = normalize(drone.position)
            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls
