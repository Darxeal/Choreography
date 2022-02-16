import math
from typing import List

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
from choreography.utils.vector_math import distance, lerp, smoothlerp
from rlutilities.linear_algebra import vec3, vec2, look_at, normalize, dot, axis_to_rotation, norm, xy, clip
from rlutilities.simulation import Ball, Game

Game.set_mode("soccar")


class HideBall(StateSettingStep):
    def set_ball_state(self, ball: Ball):
        ball.position = vec3(0, 0, -200)


def radius(i: int) -> float:
    return (hash(i * 2.98321) % 700) / 1000 + 0.3


# radius_to_speed = interpolate.interp1d([0.3, 0.5, 0.6, 1], [2.5, 1.5, 0.7, 0.5])
# radius_to_speed = lambda r: 2.5 - r * 2
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
    # duration = 20.0
    duration = 0

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


class Tornado2(GroupStep):
    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        result.car_states = {}
        for drone in drones:
            tornado_radius = interpolate.interp1d([0, 1000, 1500, 2000], [1000, 1200, 1500, 2000])

            if not drone.jumped:
                drone.drive.target = dot(axis_to_rotation(vec3(z=0.1)), drone.position) * 0.98
                drone.drive.speed = interpolate.interp1d([0, 2000, 10000], [2000, 1000, 500])(norm(xy(drone.position)))
                drone.drive.step(self.dt)
                drone.controls = drone.drive.controls
                if norm(drone.position) < tornado_radius(0):
                    drone.controls.jump = True

            elif norm(xy(drone.position)) < tornado_radius(1800):
                orbit = dot(axis_to_rotation(vec3(z=0.1)),
                            normalize(xy(drone.position)) * tornado_radius(drone.position.z))
                vel = normalize(orbit - xy(drone.position)) * min(norm(drone.velocity) * (1 + self.dt), 2200)
                vel.z = 300
                result.car_states[drone.id] = CarState(Physics(
                    velocity=vec3_to_vector3(vel),
                    rotation=mat3_to_rotator(look_at(
                        forward=dot(axis_to_rotation(vec3(z=0.2)), vel),
                        up=vec3(z=norm(drone.position)) - drone.position
                    )),
                    angular_velocity=vec3_to_vector3(vec3(z=1.0)),
                ), boost_amount=100)
                drone.controls.boost = True
            else:
                drone.controls.boost = False

        return result


class Tornado(PerDroneStep):
    duration = 60.0
    jump_interval = 0.5
    tornado_speed_add = 1.0

    @classmethod
    def jump_time(cls, index: int) -> float:
        return index * cls.jump_interval + wait_between_drifting_and_tornado + DetourForBigPads.duration

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        if self.time_since_start > self.jump_time(index):
            time_since_jump = self.time_since_start - self.jump_time(index)
            tornado_additional_t = time_since_jump * self.tornado_speed_add * (1 - radius(index))
            t = self.time_since_start * elliptic_speed_multiplier + tornado_additional_t
            pos = circular(index, t) * 4000
            drone.hover.target = pos
            drone.hover.target.z = hash(index * 1.1) % 1000 + 400
            drone.hover.up = dot(axis_to_rotation(drone.forward() * 0.4), drone.up())
            drone.hover.step(self.dt)
            drone.controls = drone.hover.controls

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        result.car_states = {drone.id: CarState(boost_amount=100) for drone in drones
                             if self.time_since_start > self.jump_time(drone.id)}
        return result


def wave(x: float) -> float:
    """Sinus but goes from 0 to 1 to 0 ..."""
    return (math.sin((2 * x + 3) * math.pi / 2) + 1) / 2


def smooth_start(x: float) -> float:
    """Starts smooth as a quadratic function and transitions into a linear function"""
    # return x ** 2 if x < 0.5 else x - 0.25
    return (0.5 * x) ** 2 if x < 2 else x - 1


class StateSettingTornado(GroupStep):
    duration = 60.0

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
                additional_t = smooth_start(time_since_jump * 1.0)
                t = self.time_since_start * elliptic_speed_multiplier + additional_t - 0.15

                target_height = 300 + wave(time_since_jump * 0.05) * 1700
                target_radius = smoothlerp(ellipse_radius.x, 500 + target_height * 0.7, min(time_since_jump * 0.2, 1))

                def tornado_pos(add):
                    c = circular(drone.id, t + add)
                    return smoothlerp(c, normalize(c), min((time_since_jump + add) * 0.2, 0.6)) * target_radius

                pos = tornado_pos(0)
                pos_ahead = tornado_pos(0.5)

                pos.z = pos_ahead.z = lerp(20, target_height, min(time_since_jump * 0.2, 1))

                target_up = normalize(vec3(z=norm(drone.position)) - drone.position)
                orientation = look_at(pos_ahead - drone.position,
                                      lerp(vec3(z=1), target_up, min(time_since_jump, 1)))
                # if time_since_jump > 0.1:
                #     drone.reorient.target_orientation = orientation
                #     drone.reorient.step(self.dt)
                #     drone.controls = drone.reorient.controls

                car_states[drone.id] = CarState(Physics(
                    location=vec3_to_vector3(
                        # lerp(self.jump_positions[drone.id], pos, clip(time_since_jump - 0.2, 0, 1))
                        # pos
                        lerp(drone.position, pos, 0.01)
                    ),
                    rotation=mat3_to_rotator(orientation),
                    velocity=vec3_to_vector3((pos - drone.position) / self.dt)
                ), boost_amount=100)

                drone.controls.boost = True
                drone.controls.jump = True
            else:
                car_states[drone.id] = CarState(boost_amount=100)

        result.car_states = car_states
        return result


class ExplodeBall(StateSettingStep):
    def set_ball_state(self, ball: Ball):
        ball.position = vec3(0, 5300, 300)


class DaciaAirshow(Choreography):
    map_name = "ChampionsField"

    def generate_sequence(self):
        self.sequence = [
            WaitKickoffCountdown(),
            HideBall(),
            DistributeElliptic(),
            ParallelStep([
                DriveElliptic(),
                CompositeStep([
                    DetourForBigPads(),
                    Wait(wait_between_drifting_and_tornado),
                    StateSettingTornado(),
                ], shared_start_time=True)
            ]),
        ]

    @staticmethod
    def get_num_bots() -> int:
        return 128

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        return ['dacia.cfg'] * num_bots
