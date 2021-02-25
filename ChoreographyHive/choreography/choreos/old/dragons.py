from math import pi
from typing import List

from rlbot.utils.rendering.rendering_manager import RenderingManager
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlutilities.linear_algebra import vec3, dot, look_at, axis_to_rotation, cross
from rlutilities.simulation import Ball, Input, Curve

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, StateSettingStep, ParallelStep, DroneListStep
from choreography.paths.dragon_paths import BLUE_DRAGON_PATH, PURPLE_DRAGON_PATH
from choreography.utils.vector_math import direction

TOTAL_SPEED = 0.7


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


class Dragon(StateSettingStep):
    duration = 65.0 / TOTAL_SPEED
    distance_between_body_parts = 300
    path: Curve = None
    RENDER_PATH = False

    def set_drone_states(self, drones: List[Drone]):
        # get the length of the path from start to the dragons's head (first bot)
        head_t = self.time_since_start / self.duration * self.path.length

        for drone in drones:
            # offset the other body parts
            drone_t = head_t - self.distance_between_body_parts * (drone.id - drones[0].id)

            # if we're not on the path yet, don't do any state setting
            if drone_t < 0:
                continue

            t = self.path.length - drone_t  # because Curve.point_at's argument means distance to end

            pos = self.path.point_at(t)
            pos_ahead = self.path.point_at(t - 500)
            pos_behind = self.path.point_at(t + 300)

            # figure out the orientation of the body part
            facing_direction = direction(pos_behind, pos)
            target_left = cross(facing_direction, direction(pos, pos_ahead))
            target_up = cross(target_left, facing_direction)
            up = drone.up() + target_up * 0.9 + vec3(0, 0, 0.1)
            target_orientation = look_at(facing_direction, up)

            drone.position = pos_behind
            drone.velocity = facing_direction * (self.path.length / self.duration)
            drone.angular_velocity = vec3(0, 0, 0)  # TODO: setting correct angular velocity could help with replays
            drone.orientation = target_orientation

    def render(self, renderer: RenderingManager):
        if self.RENDER_PATH:
            renderer.draw_polyline_3d(self.path.points[::5], renderer.white())


class BlueDragon(Dragon):
    path = Curve(BLUE_DRAGON_PATH.to_points(2000))
    target_indexes = range(0, 5)


class PurpleDragon(Dragon):
    path = Curve(PURPLE_DRAGON_PATH.to_points(2000))
    target_indexes = range(5, 10)


class RingOfFire(DroneListStep):
    ring_radius = 500
    orbit_radius = 2200
    orbit_center = vec3(0, 0, 1400)
    starting_orbit_rotation: float = None
    orbit_start_delay = 20 / TOTAL_SPEED
    orbit_speed = 0.45 * TOTAL_SPEED

    def step(self, packet: GameTickPacket, drones: List[Drone]):
        orbit_t = max(0, self.time_since_start - self.orbit_start_delay)
        orbit_rotation = self.starting_orbit_rotation + orbit_t * self.orbit_speed

        direction_from_center = dot(axis_to_rotation(vec3(0, 0, 1) * orbit_rotation), vec3(1, 0, 0))
        ring_center = self.orbit_center + direction_from_center * self.orbit_radius
        ring_facing_direction = cross(direction_from_center, vec3(0, 0, 1))

        for drone in drones:
            i = drone.id - drones[0].id
            angle = i / len(drones) * pi * 2
            pos = ring_center + dot(vec3(0, 0, 1), axis_to_rotation(ring_facing_direction * angle)) * self.ring_radius

            if pos[2] > self.orbit_center[2] + self.ring_radius - self.time_since_start * 200:
                drone.hover.target = pos
                drone.hover.up = ring_facing_direction
                drone.hover.step(self.dt)
                drone.controls = drone.hover.controls
                drone.controls.jump = True


class Ring1(RingOfFire):
    target_indexes = range(10, 25)
    starting_orbit_rotation = pi


class Ring2(RingOfFire):
    target_indexes = range(25, 40)
    starting_orbit_rotation = 0


class DragonBoost(BlindBehaviorStep):
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
                DragonBoost(),
                Ring1(),
                Ring2()
            ]),
        ]
