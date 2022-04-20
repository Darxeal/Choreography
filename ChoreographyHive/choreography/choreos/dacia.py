import math
import random
from abc import ABC
from bisect import bisect_left
from typing import List

from perlin_noise import PerlinNoise
from rlbot.utils.game_state_util import CarState, Physics
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface
from scipy import interpolate

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.steps.base_steps import StateSettingStep, PerDroneStep, DroneListStep, StepResult, vec3_to_vector3, \
    mat3_to_rotator, GroupStep
from choreography.steps.higher_order import ParallelStep, CompositeStep
from choreography.steps.utils import WaitKickoffCountdown, Wait
from choreography.utils.vector_math import distance, lerp, smootherlerp, sigmoid, invlerp, smoothstep
from rlutilities.linear_algebra import vec3, vec2, look_at, normalize, dot, axis_to_rotation, norm, xy, clip, \
    angle_between, cross, rotation, eye3, mat3
from rlutilities.simulation import Ball, Game, Field, ray, Car, sphere

Game.set_mode("soccar")


class ChargeBallWithGoalExplosion(StateSettingStep):
    def set_ball_state(self, ball: Ball):
        ball.position = vec3(0, 0, 200)
        ball.velocity = vec3(0, 0, -1)

    def set_drone_states(self, drones: List[Drone]):
        drones[0].position = vec3(0, 0, 50)
        drones[0].velocity = vec3(0, 0, 0)


class HideBall(StateSettingStep):
    def set_ball_state(self, ball: Ball):
        ball.position = vec3(0, 0, -1000)
        ball.velocity = vec3(0, 0, 0)
        ball.angular_velocity = vec3(0, 0, 0)


class HideDrones(StateSettingStep):
    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            drone.position.z = 17
            drone.position.x = 4000 + (drone.id // 5) * 150 - 9800
            drone.position.y = 5000 + (drone.id % 5) * 150


class HideDronesContinuous(StateSettingStep):
    duration = math.inf

    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            drone.position.z = 5000
            drone.position.x = 100000
            drone.position.y = 0


class BillboardBall(StateSettingStep):
    def set_ball_state(self, ball: Ball):
        ball.position = vec3(22_620, -10_000, 5_700)
        ball.velocity = vec3(0, 0, 0)
        ball.angular_velocity = vec3(0, 0, 0)


tornado_layer_counts = [7, 6] + [5] * 2 + [4] * 5 + [3] * 7
assert sum(tornado_layer_counts) == 64
tornado_cumulative_layer_counts = [sum(tornado_layer_counts[:i + 1]) for i in range(len(tornado_layer_counts))]

tornado_group_index = [bisect_left(tornado_cumulative_layer_counts, i + 1) for i in range(64)]
tornado_index_in_group = [tornado_layer_counts[tornado_group_index[i]] -
                          tornado_cumulative_layer_counts[tornado_group_index[i]] + i for i in range(64)]


def ground_tornado_radius(i: int) -> float:
    return tornado_group_index[i] / len(tornado_layer_counts) * 0.7 + 0.3


def tornado_unit_offset(i: int, t: float) -> vec3:
    a = tornado_index_in_group[i] / tornado_layer_counts[tornado_group_index[i]] * math.pi * 2
    r = ground_tornado_radius(i)
    s = 1 - r * 0.5
    t += 10000
    return vec3(math.cos(a + t * s) * r, math.sin(a + t * s) * r, 0)


ground_tornado_radius_scale = 3500


def ground_tornado_offset(i: int, t: float) -> vec3:
    return tornado_unit_offset(i, t) * ground_tornado_radius_scale


class DistributeGroundTornado(StateSettingStep):
    def set_drone_states(self, drones: List[Drone]):
        for i in range(len(drones)):
            drones[i].position = ground_tornado_offset(i, 0)
            drones[i].position.z = 30
            drones[i].orientation = look_at(ground_tornado_offset(i, 0.1) - ground_tornado_offset(i, 0))
            drones[i].angular_velocity = vec3()
            drones[i].velocity = vec3()
            drones[i].boost = 0


ground_tornado_speed = 0.6


class GroundTornadoDrive(PerDroneStep):
    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        pos = ground_tornado_offset(index, self.time_since_start * ground_tornado_speed)
        drone.drive.target = pos
        drone.drive.speed = distance(drone.position, pos) * 3.0
        drone.drive.step(self.dt)
        drone.controls = drone.drive.controls

        drone.controls.handbrake = False
        if angle_between(drone.forward(), pos - drone.position) > 1.5:
            drone.controls.handbrake = True

        if not drone.on_ground:
            drone.controls.boost = False


LARGE_PADS = {
    3: vec3(-3072.0, -4096.0, 0.0),
    4: vec3(3072.0, -4096.0, 0.0),
    15: vec3(-3584.0, 0.0, 0.0),
    18: vec3(3584.0, 0.0, 0.0),
    29: vec3(-3072.0, 4096.0, 0.0),
    30: vec3(3072.0, 4096.0, 0.0),
}


class DetourForBigPads(DroneListStep):
    duration = 25

    def step(self, packet: GameTickPacket, drones: List[Drone]):
        if self.time_since_start < 2.0:
            return
        for pad_index, pad_position in LARGE_PADS.items():
            if packet.game_boosts[pad_index].is_active:
                nearest_drone = min(drones, key=lambda drone: distance(drone.position + drone.velocity, pad_position))
                if ground_tornado_radius(nearest_drone.id) > 0.9:
                    if distance(pad_position, nearest_drone.position) > 600:
                        nearest_drone.drive.target = pad_position
                        nearest_drone.drive.step(self.dt)
                        nearest_drone.controls = nearest_drone.drive.controls
                        nearest_drone.controls.boost = False
                    else:
                        nearest_drone.controls.handbrake = True
                        nearest_drone.controls.boost = False


wait_between_drifting_and_tornado = 3.0


def smooth_start(x: float) -> float:
    """Starts smooth as a quadratic function and transitions into a linear function"""
    return x ** 2 if x < 0.5 else x - 0.25


random.seed(42)
random_values = [random.random() for _ in range(64)]


class Tornado(GroupStep):
    duration = 45.0 + DetourForBigPads.duration

    jump_interval = 2.0

    @classmethod
    def jump_time(cls, index: int) -> float:
        return index * cls.jump_interval + wait_between_drifting_and_tornado + DetourForBigPads.duration

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        car_states = {}
        for drone in drones:
            drone_group_id = tornado_group_index[drone.id]
            time_since_jump = self.time_since_start - self.jump_time(drone_group_id)
            if time_since_jump > 0:
                target_height = 400 + (1 - drone_group_id / 16) * 1400 * smoothstep(0, 1,
                                                                                    min(time_since_jump * 0.2, 1))

                tornado_radius = 400 + (target_height / 1800) ** 2 * 1800 * 0.8
                target_radius = smootherlerp(
                    a=ground_tornado_radius_scale,
                    b=tornado_radius,
                    t=min(time_since_jump * 0.2, 1)
                )

                additional_t = smooth_start((1 + drone_group_id / 16 * 5) * max(0, time_since_jump - 1.0) * 0.5)
                additional_t = smooth_start(additional_t / 10) * 10

                t = self.time_since_start * ground_tornado_speed + additional_t - 0.17

                def tornado_pos(add):
                    c = tornado_unit_offset(drone.id, t + add)
                    return smootherlerp(c, normalize(c), min((time_since_jump + add) * 0.2, 1)) * target_radius

                pos = tornado_pos(0 - 0.04)
                t_ahead = 1.5
                pos_ahead = tornado_pos(t_ahead)

                pos.z = lerp(17, target_height, min(1, time_since_jump * 0.8) ** 0.5)
                pos_ahead.z = lerp(17, target_height, min(1, (time_since_jump + t_ahead) * 0.8) ** 0.5)

                target_up = normalize(vec3(z=norm(drone.position)) - drone.position)
                target_forward = normalize(pos_ahead - drone.position) + vec3(z=0.2)
                orientation = look_at(
                    lerp(drone.forward(), target_forward, min(time_since_jump * 0.5, 1)),
                    lerp(vec3(z=1), target_up, min(time_since_jump * 0.5, 1))
                )

                car_states[drone.id] = CarState(Physics(
                    location=vec3_to_vector3(pos),
                    rotation=mat3_to_rotator(orientation),
                    velocity=vec3_to_vector3((pos_ahead - pos) / t_ahead)
                ), boost_amount=100)

                drone.controls.boost = True
                drone.controls.jump = True
            else:
                car_states[drone.id] = CarState(boost_amount=100)

        result.car_states = car_states
        return result


class TeleportBallToCenter(StateSettingStep):
    duration = 0.01

    def set_ball_state(self, ball: Ball):
        ball.position = vec3(0, 0, 1000)
        ball.velocity = vec3()
        ball.angular_velocity = vec3()


class ExplodeBall(StateSettingStep):
    def set_ball_state(self, ball: Ball):
        ball.position = vec3(0, 5300, 300)


class LandSmoothlyAndTransitionToZigZag(PerDroneStep):
    duration = 3.0

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        target = vec3(dot(rotation(0.4), normalize(vec2(drone.position))) * 10_000)
        target.z = 1000
        if drone.on_ground:
            drone.drive.target = target
            drone.drive.step(self.dt)
            drone.controls = drone.drive.controls
            drone.controls.boost = True
            return

        copy = Car(drone)
        while copy.time < drone.time + 2.0:
            copy.step(drone.controls, 1 / 10)
            result = Field.collide(sphere(copy.position, 200))
            if norm(result.direction) > 0.5:
                target_forward = normalize(target - drone.position)
                landing_up = result.direction
                landing_forward = normalize(target_forward - dot(target_forward, landing_up) * landing_up)
                drone.reorient.target_orientation = look_at(landing_forward, landing_up)
                drone.reorient.step(self.dt)
                drone.controls = drone.reorient.controls
                break


class InfiniteBoost(GroupStep):
    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        result.car_states = {drone.id: CarState(boost_amount=100) for drone in drones}
        return result


class ZigZag(StateSettingStep):
    duration = 50.0
    noise = PerlinNoise(seed=42, octaves=2)

    target_indexes = range(0, 50)

    @classmethod
    def pos_at_time(cls, i: int, t: float) -> ray:
        r = lerp(10_000, 2000, t / cls.duration)
        r = max(r, 2500)
        angle = i / 50 * math.pi * 2 + t * 0.4 + t ** 2 * 0.001
        dir = vec3(math.cos(angle), math.sin(angle), 0)
        height = sigmoid(cls.noise([dir.x, dir.y, t * 0.1]) * 5) * 2000 + 10
        height = lerp(random_values[i] * 2000 + 10, height, clip(t / 8 - 0.5, 0, 1))
        if height < 1000:
            height = smootherlerp(height, 1000, 1 - clip(abs(dir.x * 2), 0, 1))
        start = vec3(0, 0, height)
        dir *= r
        return Field.raycast_nearest(ray(start, dir))

    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            coll = self.pos_at_time(drone.id, self.time_since_start)
            t_ahead = 0.5
            coll_ahead = self.pos_at_time(drone.id, self.time_since_start + t_ahead)

            on_wall = norm(coll.direction) > 0
            pos = coll.start + coll.direction * (17)

            landing_t = clip((self.time_since_start - 45) * 0.3, 0, 1)
            drone.position = lerp(pos, xy(pos) + vec3(z=17), landing_t)
            forward = coll_ahead.start - coll.start
            forward = lerp(forward, xy(forward), landing_t)
            if on_wall:
                forward -= coll.direction * 15
            up = coll.direction if on_wall else vec3(z=1) - normalize(drone.position)
            up = lerp(up, vec3(z=1), landing_t)

            drone.orientation = look_at(forward, up)
            drone.velocity = vec3()  # forward / t_ahead
            drone.boost = 100
            drone.controls.boost = True
            drone.controls.throttle = True

            ahead_local = dot(coll_ahead.start - coll.start, drone.orientation)
            drone.controls.steer = clip(ahead_local.y * 300, -1, 1)
            drone.controls.handbrake = True


bolt_points = [
    vec2(-0.3, 1.0),
    vec2(-0.1, 0.2),
    vec2(-0.7, 0.2),
    vec2(0.3, -1.0),
    vec2(0.1, -0.2),
    vec2(0.7, -0.2),
]
bolt_points = [p * 3500 for p in bolt_points]

bolt_dists = [0]
for i in range(len(bolt_points)):
    bolt_dists.append(bolt_dists[-1] + distance(bolt_points[i - 1], bolt_points[i]))

bolt_points_closed = [bolt_points[-1]] + bolt_points


class LightningBolt(StateSettingStep):
    speed = 2000
    intervals = 1250
    target_indexes = range(17)
    duration = 20.0

    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            dist = (self.time_since_start * self.speed + drone.id * self.intervals) % bolt_dists[-1]
            bolt_index = bisect_left(bolt_dists, dist) - 1
            t = invlerp(bolt_dists[bolt_index], bolt_dists[bolt_index + 1], dist)
            a = bolt_points_closed[bolt_index]
            b = bolt_points_closed[bolt_index + 1]
            c = bolt_points[(bolt_index + 1) % len(bolt_points)]

            pos_on_bolt = smootherlerp(a, b, t)

            angle = dist / bolt_dists[-1] * math.pi * 2
            pos_on_circle = vec2(math.cos(angle), math.sin(angle)) * 2500

            start_t = clip(self.time_since_start * 0.2 - 0.3, 0, 1)
            end_t = clip((self.duration - self.time_since_start) * 0.3 - 1.0, 0, 1)
            pos = vec3(lerp(pos_on_circle, pos_on_bolt, min(start_t, end_t)))
            pos.z = 19
            drone.position = pos

            forward = vec3(normalize(b - a))
            forward_next = vec3(normalize(c - b))
            ori_t = interpolate.interp1d([0, 0.3, 1], [0, 0, 1])(t)
            bolt_forward = normalize(lerp(forward, forward_next, ori_t))
            circle_forward = normalize(vec3(math.cos(angle + math.pi / 2), math.sin(angle + math.pi / 2), 0))
            drone.orientation = look_at(lerp(circle_forward, bolt_forward, end_t))

            drone.velocity = vec3()
            drone.angular_velocity = vec3()

            drone.boost = 100
            drone.controls.boost = t < 0.8
            drone.controls.handbrake = True


class StateSettingFlight(StateSettingStep, ABC):
    lookahead_t = 0.5

    @classmethod
    def position(cls, t: float, i: int) -> vec3:
        raise NotImplementedError

    @classmethod
    def up(cls, t: float, pos: vec3) -> vec3:
        raise NotImplementedError

    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            pos = self.position(self.time_since_start, drone.id)
            pos_ahead = self.position(self.time_since_start + self.lookahead_t, drone.id)
            drone.position = pos
            drone.orientation = look_at(pos_ahead - pos, self.up(self.time_since_start, pos))
            drone.velocity = (pos_ahead - pos) / self.lookahead_t
            drone.boost = 100
            drone.controls.boost = True


class BoltToSpirals(StateSettingFlight):
    duration = 10.0

    ball_radius = 1000
    speed = 0.7
    sphere_portion = 0.9

    n_layers = 15
    bots_per_layer = 3
    target_indexes = range(n_layers * bots_per_layer)

    @classmethod
    def position(cls, t: float, i: int) -> vec3:
        group_index = int(i / cls.bots_per_layer)
        index_in_group = i % cls.bots_per_layer

        layer = (1 - 2 * (group_index + 0.5) / cls.n_layers) * cls.sphere_portion
        orbit_radius = (1 - layer ** 2) ** 0.5
        radius_t = clip(t * 0.3 - 1.2, 0, 1)

        start_angle = (index_in_group / cls.bots_per_layer + group_index / cls.n_layers / 3) * math.pi * 2
        angle = start_angle + t * cls.speed + (max(t * 0.3 - 1.4, 0) * 1.0) ** 2 * 0.5
        orbit = dot(axis_to_rotation(vec3(z=angle)), vec3(1, 0, 0))

        layer_height = 1000 + layer * cls.ball_radius
        height_t = clip(t * 0.3 - 1, 0, 1)

        height_offset = vec3(z=smootherlerp(19, layer_height, height_t))
        orbit_offset = orbit * smootherlerp(2500, orbit_radius * cls.ball_radius, radius_t)

        return height_offset + orbit_offset

    @classmethod
    def up(cls, t: float, pos: vec3) -> vec3:
        return lerp(vec3(z=1), normalize(vec3(0, 0, 0.5) - normalize(xy(pos))), clip(pos.z / 100 - 0.1, 0, 1))


class LightBall(StateSettingFlight):
    duration = 15.0

    sphere_radius = 1000
    sphere_portion = 0.9
    center = vec3(0, 0, 1000)
    speed = 1.5

    bots_per_layer = 2
    n_layers = 20
    target_indexes = range(n_layers * bots_per_layer)

    layer_start_offset_multiplier = -1.0
    layer_offset_speed = 0.0
    reverse_odd_layers = True

    @classmethod
    def sphere_orientation(cls, t: float) -> mat3:
        return eye3()

    @classmethod
    def layer_offset_multiplier(cls, t: float, i: int) -> float:
        return 1.0

    @classmethod
    def position(cls, t: float, i: int) -> vec3:
        group_index = int(i / cls.bots_per_layer)
        index_in_group = i % cls.bots_per_layer
        group_sgn = (1 - 2 * (group_index % 2)) if cls.reverse_odd_layers else 1

        layer = (1 - 2 * (group_index + 0.5) / cls.n_layers) * cls.sphere_portion
        orbit_radius_t = (1 - layer ** 2) ** 0.5

        layer_start_angle_offset = layer * cls.layer_start_offset_multiplier * group_sgn
        start_angle = index_in_group * math.pi * 2 / cls.bots_per_layer + layer_start_angle_offset

        angle = start_angle + t * cls.speed * (1 + layer * cls.layer_offset_speed) * group_sgn
        orbit = dot(axis_to_rotation(vec3(z=angle)), vec3(1, 0, 0))

        layer_offset = vec3(z=layer * cls.sphere_radius) * cls.layer_offset_multiplier(t, i)
        orbit_offset = orbit * orbit_radius_t * cls.sphere_radius
        sphere_offset = layer_offset + orbit_offset
        return cls.center + dot(cls.sphere_orientation(t), sphere_offset)

    @classmethod
    def up(cls, t: float, pos: vec3) -> vec3:
        return normalize(xy(cls.center) - xy(pos)) + vec3(0, 0, 0.3)


class LightBallSpirals(LightBall):
    duration = 17.0

    n_layers = 20
    bots_per_layer = 3
    target_indexes = range(n_layers * bots_per_layer)

    reverse_odd_layers = False
    layer_start_offset_multiplier = -3.0
    layer_offset_speed = 0.3

    @classmethod
    def sphere_orientation(cls, t: float) -> mat3:
        lerp_t = clip((t - 14) * 0.3, 0, 1)
        return axis_to_rotation(vec3(x=smoothstep(0, 1, lerp_t) * math.pi / 2))

    @classmethod
    def layer_offset_multiplier(cls, t: float, i: int) -> float:
        lerp_t = clip((t - 16) * 2.0, 0, 30)
        return 1 + smooth_start(lerp_t)


class Atom(StateSettingFlight):
    duration = 30.0

    orbit_normals = [
        vec3(0, 0.01, 1),
        vec3(0, 1, 0),
        vec3(0, 1, 1),
        vec3(0, -1, 1),
    ]
    orbit_delays = [-0.3, 0.3, 0.6, 0.0]
    bots_per_orbit = 3

    orbits = len(orbit_normals)
    target_indexes = range(orbits * bots_per_orbit)

    speed = 1.5
    accel = 0.05
    radius = 1000

    entire_atom_rotation_speed = 0.1

    noise = PerlinNoise(seed=42, octaves=2)

    @classmethod
    def center(cls, t: float) -> vec3:
        noise_scale = 500
        noise_coord = 0.2 * t + 0.02 * t ** 2
        return vec3(
            noise_scale * cls.noise(noise_coord),
            noise_scale * cls.noise(noise_coord + 10),
            noise_scale * cls.noise(noise_coord + 20) * 0.1 + 1000
        )

    @classmethod
    def position(cls, t: float, i: int) -> vec3:
        orbit_index = i % cls.orbits
        index_in_orbit = int(i / cls.orbits)

        orbit_normal = normalize(cls.orbit_normals[orbit_index])
        orbit_normal = dot(axis_to_rotation(vec3(z=t * cls.entire_atom_rotation_speed)), orbit_normal)

        t_offset = index_in_orbit * 2 * math.pi / cls.bots_per_orbit + cls.orbit_delays[orbit_index]
        angle = (cls.speed * t + cls.accel * t ** 2 + t_offset) % (2 * math.pi)

        orbit = normalize(cross(orbit_normal, cross(orbit_normal, vec3(z=1))))
        return cls.center(t) + dot(axis_to_rotation(orbit_normal * angle), orbit * cls.radius)

    @classmethod
    def up(cls, t: float, pos: vec3) -> vec3:
        return cls.center(t) - pos

    def set_ball_state(self, ball: Ball):
        ball.position = self.center(self.time_since_start)
        ball.velocity = vec3()
        ball.angular_velocity = vec3()


class SpiralsIntoAtom(StateSettingFlight):
    duration = LightBallSpirals.duration + 8
    target_indexes = LightBallSpirals.target_indexes

    @classmethod
    def lerp_t(cls, t: float) -> float:
        return clip(invlerp(LightBallSpirals.duration - 0, LightBallSpirals.duration + 2, t), 0, 1)

    @classmethod
    def position(cls, t: float, i: int) -> vec3:
        atom_target_indexes = range(
            len(LightBallSpirals.target_indexes) // 2 - len(Atom.target_indexes) // 2,
            len(LightBallSpirals.target_indexes) // 2 + len(Atom.target_indexes) // 2,
        )
        if i in atom_target_indexes:
            return smootherlerp(
                LightBallSpirals.position(t, i),
                Atom.position(t - LightBallSpirals.duration, i - atom_target_indexes.start),
                cls.lerp_t(t)
            )
        return LightBallSpirals.position(t, i)

    @classmethod
    def up(cls, t: float, pos: vec3) -> vec3:
        return smootherlerp(
            LightBallSpirals.up(t, pos),
            Atom.up(t - LightBallSpirals.duration, pos),
            cls.lerp_t(t)
        )


class DaciaAirshow(Choreography):
    map_name = "ChampionsField"

    def generate_sequence(self):
        self.sequence = [
            WaitKickoffCountdown(),

            ChargeBallWithGoalExplosion(),
            Wait(1.0),
            HideBall(),
            DistributeGroundTornado(),
            ParallelStep([
                GroundTornadoDrive(),
                CompositeStep([
                    DetourForBigPads(),
                    Wait(wait_between_drifting_and_tornado),
                    Tornado(),
                ], shared_start_time=True)
            ]),
            TeleportBallToCenter(),
            ExplodeBall(),
            ParallelStep([
                LandSmoothlyAndTransitionToZigZag(),
                InfiniteBoost(),
            ]),
            Wait(1.0),

            HideBall(),
            HideDrones(),
            ZigZag(),

            HideDrones(),
            LightningBolt(),

            HideDrones(),
            SpiralsIntoAtom(),

            ChargeBallWithGoalExplosion(),
            Wait(1.0),
            ParallelStep([
                HideDronesContinuous(),
                Atom(),
            ]),
            ExplodeBall(),
            Wait(5.0),
        ]

    @staticmethod
    def get_num_bots() -> int:
        return 64

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        return ['dacia.cfg'] * num_bots

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        return [0] * num_bots
