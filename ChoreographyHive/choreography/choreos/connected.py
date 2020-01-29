import math
from typing import List

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, StepResult, PerDroneStep, \
    StateSettingStep, TwoTickStateSetStep

from rlutilities.linear_algebra import vec2, vec3, euler_to_rotation, rotation, dot, normalize, norm, xy, axis_to_rotation
from rlutilities.simulation import Ball, Input

from .examples import YeetTheBallOutOfTheUniverse, Wait

class ConnectedChoreo(Choreography):
    
    @staticmethod
    def get_num_bots():
        return 64

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        # Only index 3 has test appearance.
        appearances = ['red_blue.cfg'] * 32 + ['yellow_green.cfg'] * 32
        return appearances

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        # Every other bot is on the orange team.
        teams = [0] * num_bots
        teams[16:32] = [1] * 16
        teams[48:64] = [1] * 16
        return teams

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            # FourStacks(),
            # ResetAttr(),
            # Wait(2.0),
            # ForwardThenQuadHelix(),
            # SortToCircle(),
            ResetCircle(), # Cheating while testing
            StationaryCircle(),
            SpinUp(),
            SpinDown(),
            StationaryCircle(),
            SeparateIntoFourGroups(),
            SmallerCirclesStart(),
            SmallerCirclesToCylinder(),
            SpinInCylinder()
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
        drone.start_pos = None
        drone.sort_phase = 0

class ForwardThenQuadHelix(PerDroneStep):
    delay = 0.8
    duration = 55.0

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

            # HEIGHT
            if drone.since_jumped < 16:
                height = 200 + drone.since_jumped * 80 # speed of rise
            elif drone.since_jumped < 24:
                height = 1480 - (drone.since_jumped - 16) * 80
            elif drone.since_jumped < 36:
                height = 840 + (drone.since_jumped - 24) * 80
            else:
                height = 1800 - (drone.since_jumped - 36) * 120

            # RADIUS
            if drone.since_jumped < 16:
                radius = 550 + drone.since_jumped**3 / 10
            elif drone.since_jumped < 40:
                radius = 960 - (drone.since_jumped - 16) * 20
            else:
                radius = 480

            angle = drone.since_jumped * 0.35 # rotation speed

            # Offset angle.
            angle += (index // 16) * (math.pi / 2)

            # Set hover target and controller.
            rot = rotation(angle)
            v = vec3(dot(rot, vec2(1, 0)))
            drone.hover.target = v * radius
            drone.hover.target[2] = height
            drone.hover.up = normalize(drone.position)
            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls

            if drone.since_jumped < 0.05:
                drone.controls.jump = True
                drone.controls.boost = False

class SortToCircle(PerDroneStep):
    duration = 60.0
    delay = 0.4
    radius = 1800
    height = 800

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        if drone.start_pos is None:
            drone.start_pos = drone.position
            drone.sort_phase = 1

        # Finish if all have been sorted.
        if index == 0:
            # Initially set finished to True.
            self.finished = True
        if drone.sort_phase != 4:
            # Will be set to False if any is not in phase 4.
            self.finished = False 

        # It's my time!
        if self.time_since_start > 1.0 + index * self.delay:

            desired_angle = (2 * math.pi / 64) * index

            current_radius = norm(vec2(drone.position))
            current_angle = math.atan2(drone.position[1], drone.position[0])
            if current_angle < 0.0: current_angle += 2 * math.pi # only positive angles

            # First get right radius.
            if drone.sort_phase == 1:
                if current_radius > self.radius - 300:
                    drone.sort_phase = 2
                target = normalize(vec2(drone.position)) * (current_radius + 400)
                target = vec3(target)
                target[2] = (3*drone.position[2] + self.height + 100 + (180 * index//16)) / 4 # Also adjust height
                
            # Then rotate to correct angle.
            elif drone.sort_phase == 2:
                # if index in range(64)[::8]: print(index, current_angle, desired_angle)
                if abs(current_angle - desired_angle) < 0.1:
                    drone.sort_phase = 3
                target = dot(rotation(current_angle + 0.4), vec2(self.radius - 300, 0))
                target = vec3(target)
                target[2] = self.height + 100 + (180 * index//16)

            # Height and final corrections.
            elif drone.sort_phase == 3:
                target = vec3(dot(rotation(desired_angle), vec2(self.radius, 0)))
                target[2] = self.height
                if norm(drone.position - target) < 200:
                    drone.sort_phase = 4

                target[2] = (4*drone.position[2] + self.height) / 5

            elif drone.sort_phase == 4:
                target = vec3(dot(rotation(desired_angle), vec2(self.radius, 0)))
                target[2] = self.height

        else:
            # Stay in place.
            target = drone.start_pos

        # Hover controls
        drone.hover.target = target
        drone.hover.up = normalize(drone.position)
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls

        # If any bot got lost, now they have a chance to recover.
        if drone.has_wheel_contact:
            drone.controls.jump = True
        else:
            drone.controls.jump = False

class ResetCircle(TwoTickStateSetStep):
    radius = 1800
    height = 800

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            angle = (2 * math.pi / 64) * i
            drone.position = vec3(dot(rotation(angle), vec2(self.radius, 0)))
            drone.position[2] = self.height
            drone.velocity = vec3(0, 0, 0)
            drone.orientation = euler_to_rotation(vec3(math.pi/2, angle, math.pi))
            drone.angular_velocity = vec3(0, 0, 0)

class StationaryCircle(PerDroneStep):
    duration = 2.0
    radius = 1800
    height = 800

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        angle = (2 * math.pi / 64) * index
        target = vec3(dot(rotation(angle), vec2(self.radius, 0)))
        target[2] = self.height

        # Hover controls
        drone.hover.target = target
        drone.hover.up = normalize(drone.position)
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls

        # If any bot got lost, now they have a chance to recover.
        if drone.has_wheel_contact:
            drone.controls.jump = True
        else:
            drone.controls.jump = False

class SpinUp(PerDroneStep):
    duration = 5.0
    height = 800

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        radius = 1800 - 150 * self.time_since_start
        angle = self.time_since_start * (math.pi / 5)
        angle += (2 * math.pi / 64) * index        
        target = vec3(dot(rotation(angle), vec2(radius, 0)))
        target[2] = self.height

        drone.hover.up = normalize(drone.position)
        drone.hover.target = target
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls

class SpinDown(PerDroneStep):
    duration = 5.0
    height = 800

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        radius = 1050 + 150 * self.time_since_start
        angle = math.pi + self.time_since_start * (math.pi / 5)
        angle += (2 * math.pi / 64) * index
        target = vec3(dot(rotation(angle), vec2(radius, 0)))
        target[2] = self.height

        drone.hover.up = normalize(-1 * xy(drone.position))
        drone.hover.target = target
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls

class SeparateIntoFourGroups(PerDroneStep):
    duration = 5.0
    start_radius = 1800
    end_radius = 3000
    height = 800

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        # Calculate shift direction.
        direction_angle = (math.pi / 4) + ((math.pi / 2) * (index // 16))
        direction = vec3(dot(rotation(direction_angle), vec2(1, 0)))

        # Shift in one direction.
        angle = (2 * math.pi / 64) * index
        target = vec3(dot(rotation(angle), vec2(self.start_radius, 0)))
        target += direction * (self.end_radius - self.start_radius) * (self.time_since_start / self.duration)
        target[2] = self.height

        target = (target + drone.position) / 2

        # Hover controls.
        drone.hover.up = normalize(drone.position)
        drone.hover.target = target
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls

class SmallerCirclesStart(PerDroneStep):
    duration = 5.0
    small_radius = 600
    big_radius = 2700
    height = 800
    
    def step(self, packet: GameTickPacket, drone: Drone, index: int):
            
        # Get centre of small circle.
        direction_angle = (math.pi / 4) + ((math.pi / 2) * (index // 16))
        centre = vec3(dot(rotation(direction_angle), vec2(1, 0))) * self.big_radius
        centre[2] = self.height

        # Angle for the small circle.
        angle = (2 * math.pi / 16) * (index % 16)
        angle += (2 * math.pi / 4) * (index // 16 - 1) 
        target = vec3(dot(rotation(angle), vec2(self.small_radius, 0)))
        target += centre

        # Try to ease the transition
        target[2] += (index % 16 - 8) * (20 - (20/5 * self.time_since_start))

        # Hover controls.
        drone.hover.up = normalize(drone.position - centre)
        drone.hover.target = target
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls

        # If any bot got lost, now they have a chance to recover.
        if drone.has_wheel_contact:
            drone.controls.jump = True
        else:
            drone.controls.jump = False

class SmallerCirclesToCylinder(PerDroneStep):
    small_radius = 600
    big_radius = 2700

    height = 800
    height_diff = 150

    speed = 250
    spin = 0.4

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        self.finished = (100 + self.big_radius - self.time_since_start * self.speed) <= 0

        # Get centre of small circle.
        direction_angle = (math.pi / 4) + ((math.pi / 2) * (index // 16))
        centre = vec3(dot(rotation(direction_angle), vec2(1, 0)))
        # Crossing action.
        centre *= (self.big_radius - self.time_since_start * self.speed)
        centre[2] = self.height

        # Angle for the small circle.
        angle = (2 * math.pi / 16) * (index % 16)
        angle += (2 * math.pi / 4) * (index // 16 - 1)
        angle += self.time_since_start * self.spin
        target = vec3(dot(rotation(angle), vec2(self.small_radius, 0)))
        target += centre
        
        # Different heights.
        target[2] += (index // 16 - 2) * self.height_diff

        # Hover controls.
        drone.hover.up = normalize(drone.position - centre)
        drone.hover.target = target
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls

class SpinInCylinder(PerDroneStep):
    duration = 10.0

    radius = 600

    height = 800
    height_diff = 150

    spin = 0.3

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        
        drone.hover.up = normalize(drone.position)
        clockwise_rotation = axis_to_rotation(vec3(0, 0, self.spin * (-1)**(index // 16)))
        position_on_circle = normalize(xy(drone.position)) * self.radius
        drone.hover.target = dot(clockwise_rotation, position_on_circle)
        drone.hover.target[2] = self.height
        drone.hover.target[2] += (index // 16 - 2) * self.height_diff
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls
