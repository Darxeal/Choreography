from math import pi
from typing import List

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlutilities.linear_algebra import vec3, dot, look_at, axis_to_rotation, cross
from rlutilities.simulation import Ball, Input, Curve

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, StateSettingStep, ParallelStep, PerDroneStep
from choreography.paths.dragon_paths import BLUE_DRAGON_PATH, PURPLE_DRAGON_PATH
from choreography.utils.vector_math import direction


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
