import math
from typing import List

from perlin_noise import PerlinNoise
from rlbot.utils.game_state_util import CarState, Physics, Vector3
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface
from scipy import interpolate

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.steps.base_steps import StateSettingStep, PerDroneStep, DroneListStep, StepResult, vec3_to_vector3, \
    mat3_to_rotator, GroupStep
from choreography.steps.higher_order import ParallelStep, CompositeStep
from choreography.steps.utils import WaitKickoffCountdown, Wait
from choreography.utils.vector_math import distance, lerp, smoothlerp, sigmoid
from rlutilities.linear_algebra import vec3, vec2, look_at, normalize, dot, axis_to_rotation, norm, xy, clip, \
    angle_between, cross
from rlutilities.simulation import Ball, Game, Field, ray

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


def radius(i: int) -> float:
    return (hash(i * 2.98321) % 700) / 1000 + 0.3


radius_to_speed = lambda r: 1 - r * 0.5


def circular(i: int, t: float) -> vec3:
    a = hash(i * 1.65464) % (math.pi * 2)
    r = radius(i)
    s = radius_to_speed(r)
    return vec3(math.cos(a + t * s) * r, math.sin(a + t * s) * r, 0)


# ellipse_radius = vec2(3500, 4500)
ellipse_radius = vec2(3500, 3500)


def elliptic(i: int, t: float) -> vec3:
    c = circular(i, t)
    return vec3(c.x * ellipse_radius.x, c.y * ellipse_radius.y, 0)


class DistributeElliptic(StateSettingStep):
    def set_drone_states(self, drones: List[Drone]):
        for i in range(len(drones)):
            drones[i].position = elliptic(i, 0)
            drones[i].position.z = 30
            drones[i].orientation = look_at(elliptic(i, 0.1) - elliptic(i, 0))
            drones[i].angular_velocity = vec3()
            drones[i].velocity = vec3()
            drones[i].boost = 0


elliptic_speed_multiplier = 0.6


class DriveElliptic(PerDroneStep):
    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        pos = elliptic(index, self.time_since_start * elliptic_speed_multiplier)
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
    duration = 20.0

    # duration = 0

    def step(self, packet: GameTickPacket, drones: List[Drone]):
        for pad_index, pad_position in LARGE_PADS.items():
            if packet.game_boosts[pad_index].is_active:
                nearest_drone_index = min(range(len(drones)),
                                          key=lambda i: distance(drones[i].position + drones[i].velocity, pad_position))
                nearest_drone = drones[nearest_drone_index]
                if distance(pad_position, nearest_drone.position) > 600:
                    nearest_drone.drive.target = pad_position
                    nearest_drone.drive.step(self.dt)
                    nearest_drone.controls = nearest_drone.drive.controls
                    nearest_drone.controls.boost = False
                else:
                    nearest_drone.controls.handbrake = True
                    nearest_drone.controls.boost = False


wait_between_drifting_and_tornado = 3.0


def wave(x: float) -> float:
    """Sinus but goes from 0 to 1 to 0 ..."""
    return (math.sin((2 * x + 3) * math.pi / 2) + 1) / 2


def smooth_start(x: float) -> float:
    """Starts smooth as a quadratic function and transitions into a linear function"""
    # return x ** 2 if x < 0.5 else x - 0.25
    return (0.5 * x) ** 2 if x < 2 else x - 1


class Tornado(GroupStep):
    duration = 80.0

    jump_interval = 0.5

    @classmethod
    def jump_time(cls, index: int) -> float:
        return index * cls.jump_interval + wait_between_drifting_and_tornado + DetourForBigPads.duration

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        car_states = {}
        for drone in drones:
            time_since_jump = self.time_since_start - self.jump_time(drone.id)
            if time_since_jump > 0:
                additional_t = smooth_start(time_since_jump * 1.5)
                t = self.time_since_start * elliptic_speed_multiplier + additional_t - 0.15

                target_height = 300 + wave(time_since_jump * 0.05) * 1500
                target_radius = smoothlerp(ellipse_radius.x, 500 + target_height * 0.7, min(time_since_jump * 0.2, 1))

                def tornado_pos(add):
                    c = circular(drone.id, t + add)
                    return smoothlerp(c, normalize(c), min((time_since_jump + add) * 0.2, 0.6)) * target_radius

                pos = tornado_pos(0)
                pos_ahead = tornado_pos(1.5)

                pos.z = pos_ahead.z = lerp(20, target_height, min(time_since_jump * 0.2, 1))

                target_up = normalize(vec3(z=norm(drone.position)) - drone.position)
                orientation = look_at(pos_ahead - drone.position,
                                      lerp(vec3(z=1), target_up, min(time_since_jump, 1)))

                car_states[drone.id] = CarState(Physics(
                    location=vec3_to_vector3(lerp(drone.position, pos, 0.01)),
                    rotation=mat3_to_rotator(orientation),
                    velocity=vec3_to_vector3((pos - drone.position) / self.dt)
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


class InfiniteBoost(GroupStep):
    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        result.car_states = {drone.id: CarState(boost_amount=100) for drone in drones}
        return result


class Flow(StateSettingStep):
    duration = 30.0
    noise = PerlinNoise(seed=42, octaves=2)

    target_indexes = range(0, 50)

    @classmethod
    def pos_at_time(cls, i: float, t: float) -> ray:
        angle = i * math.pi * 2 + t * 0.25
        dir = vec3(math.cos(angle), math.sin(angle), 0)
        height = sigmoid(cls.noise([dir.x, dir.y, t * 0.1]) * 5) * 2000 + 10
        corrected_height = smoothlerp(height, 1000, 1 - clip(abs(dir.x * 2), 0, 1))
        start = vec3(0, 0, corrected_height)
        dir *= 10_000
        return Field.raycast_nearest(ray(start, dir))

    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            coll = self.pos_at_time(drone.id / len(drones), self.time_since_start)
            t_ahead = 0.5
            coll_ahead = self.pos_at_time(drone.id / len(drones), self.time_since_start + t_ahead)

            on_wall = norm(coll.direction) > 0

            drone.position = coll.start + coll.direction * (17)
            forward = coll_ahead.start - coll.start
            if on_wall:
                forward -= coll.direction * 15

            drone.orientation = look_at(forward, coll.direction)
            drone.velocity = forward / t_ahead
            drone.boost = 100
            drone.controls.boost = True
            drone.controls.throttle = True

            ahead_local = dot(coll_ahead.start - coll.start, drone.orientation)
            drone.controls.steer = clip(ahead_local.y * 300, -1, 1)
            drone.controls.handbrake = True


class HideDrones(StateSettingStep):
    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            drone.position.z = 17
            drone.position.x = 4000 + (drone.id // 5) * 200 - 700
            drone.position.y = 5000 + (drone.id % 5) * 200


bolt_points = [
    vec2(-0.3, 1.0),
    vec2(-0.1, 0.25),
    vec2(-0.7, 0.25),
    vec2(0.3, -1.0),
    vec2(0.1, -0.25),
    vec2(0.7, -0.25),
]
bolt_points = [vec3(p) * 3000 for p in bolt_points]
bolt_setup = [
    (lerp(bolt_points[0], bolt_points[1], 0), 0),
    (lerp(bolt_points[0], bolt_points[1], .9), 0),
    (lerp(bolt_points[1], bolt_points[2], 0), 1),
    (lerp(bolt_points[1], bolt_points[2], .9), 1),
    (lerp(bolt_points[2], bolt_points[3], 0), 2),
    (lerp(bolt_points[2], bolt_points[3], .3), 2),
    (lerp(bolt_points[2], bolt_points[3], .6), 2),
    (lerp(bolt_points[2], bolt_points[3], .96), 2),
    (lerp(bolt_points[3], bolt_points[4], 0), 3),
    (lerp(bolt_points[3], bolt_points[4], .9), 3),
    (lerp(bolt_points[4], bolt_points[5], 0), 4),
    (lerp(bolt_points[4], bolt_points[5], .9), 4),
    (lerp(bolt_points[5], bolt_points[0], 0), 5),
    (lerp(bolt_points[5], bolt_points[0], .3), 5),
    (lerp(bolt_points[5], bolt_points[0], .6), 5),
    (lerp(bolt_points[5], bolt_points[0], .96), 5),
]


class SetupBolt(StateSettingStep):
    target_indexes = range(len(bolt_setup))

    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            drone.position, last_point_index = bolt_setup[drone.id]
            next_point = bolt_points[(last_point_index + 1) % 6]
            drone.orientation = look_at(next_point - drone.position)
            drone.position.z = 17
            drone.velocity = vec3()
            drone.angular_velocity = vec3()
            drone.boost = 100


class Bolt(PerDroneStep):
    duration = 20.0
    target_indexes = range(len(bolt_setup))

    def __init__(self):
        super().__init__()
        self.current_target_indices = {i: (bolt_setup[i][1] + 1) % 6 for i in self.target_indexes}

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        target_point = bolt_points[self.current_target_indices[index]]
        if distance(drone.position, target_point) < max(norm(drone.velocity) * 0.5, 500):
            if drone.on_ground:
                if norm(drone.velocity) > 500:
                    drone.controls.throttle = -1
                    drone.controls.boost = False
                    drone.controls.steer = 0
                else:
                    drone.controls.jump = True
            else:
                self.current_target_indices[index] = (self.current_target_indices[index] + 1) % 6
        elif not drone.on_ground:
            drone.reorient.target_orientation = look_at(target_point - drone.position)
            drone.reorient.step(self.dt)
            drone.controls = drone.reorient.controls
            drone.controls.jump = False
        else:
            drone.drive.target = target_point
            drone.drive.step(self.dt)
            drone.controls = drone.drive.controls
            drone.controls.boost = True


class Atom(StateSettingStep):
    duration = 30.0

    orbit_normals = [
        vec3(0, 0.01, 1),
        vec3(0, 1, 0),
        vec3(0, 1, 1),
        vec3(0, -1, 1),
    ]
    bots_per_orbit = 3
    center = vec3(0, 0, 1000)

    orbits = len(orbit_normals)
    target_indexes = range(orbits * bots_per_orbit)

    speed = 1.5
    accel = 0.05
    radius = 1000

    @classmethod
    def pos_at_time(cls, t: float, i: int) -> vec3:
        orbit_normal = normalize(cls.orbit_normals[i % cls.orbits])
        t_offset = int(i / cls.orbits) * 2 * math.pi / cls.bots_per_orbit + (i % cls.orbits) * 5.63
        angle = (cls.speed * t + cls.accel * t ** 2 + t_offset) % (2 * math.pi)
        radius = cls.radius  # - t * 10
        orbit = normalize(cross(orbit_normal, cross(orbit_normal, vec3(z=1))))
        return cls.center + dot(axis_to_rotation(orbit_normal * angle), orbit * radius)

    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            pos = self.pos_at_time(self.time_since_start, drone.id)
            t_ahead = 0.5
            pos_ahead = self.pos_at_time(self.time_since_start + t_ahead, drone.id)
            drone.position = pos
            drone.orientation = look_at(pos_ahead - pos, self.center - pos)
            drone.velocity = (pos_ahead - pos) / t_ahead
            drone.boost = 100
            drone.controls.boost = True


class LightBall(Atom):
    orbit_normals = [
        vec3(0, 0.01, 1),
        vec3(0, 1, 0),
        vec3(0, 1, 1),
        vec3(0, -1, 1),
        vec3(1, 0, 0),
        vec3(1, 1, 0),
        vec3(1, -1, 0),
        vec3(-1, 1, 0),
        vec3(1, 0, 1),
        vec3(-1, 0, 1),
        vec3(1, 1, 1),
        vec3(1, -1, 1),
        vec3(-1, 1, 1),
        vec3(-1, -1, 1),
    ]
    orbits = len(orbit_normals)
    bots_per_orbit = 3
    target_indexes = range(orbits * bots_per_orbit)

    speed = 3.0
    # speed = 1.0
    accel = 0.0


class DaciaAirshow(Choreography):
    map_name = "ChampionsField"

    def generate_sequence(self):
        self.sequence = [
            WaitKickoffCountdown(),

            ChargeBallWithGoalExplosion(),
            Wait(1.0),
            HideBall(),
            DistributeElliptic(),
            ParallelStep([
                DriveElliptic(),
                CompositeStep([
                    DetourForBigPads(),
                    Wait(wait_between_drifting_and_tornado),
                    Tornado(),
                ], shared_start_time=True)
            ]),
            TeleportBallToCenter(),
            ExplodeBall(),
            Wait(5.0),

            HideBall(),
            HideDrones(),
            Flow(),

            HideBall(),
            HideDrones(),
            SetupBolt(),
            ParallelStep([
                Bolt(),
                InfiniteBoost(),
            ]),

            HideDrones(),
            LightBall(),

            ChargeBallWithGoalExplosion(),
            HideDrones(),
            TeleportBallToCenter(),
            Atom(),
            ExplodeBall(),
            Wait(10.0),
        ]

    @staticmethod
    def get_num_bots() -> int:
        return 64

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        return ['dacia.cfg'] * num_bots
