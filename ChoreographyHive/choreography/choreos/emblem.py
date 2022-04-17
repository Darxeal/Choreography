import math
import random
from typing import List

import numpy
from rlbot.utils.game_state_util import CarState, Physics
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface
from scipy.optimize import linear_sum_assignment

from choreography.choreography import Choreography
from choreography.choreos.dacia import HideDrones, HideBall
from choreography.drone import Drone
from choreography.steps.base_steps import StateSettingStep, GroupStep, StepResult, vec3_to_vector3, PerDroneStep
from choreography.steps.higher_order import ParallelStep, CompositeStep
from choreography.steps.utils import Wait
from choreography.utils.img_to_shape import convert_img_to_shape
from choreography.utils.vector_math import smootherlerp, distance, smootherstep
from rlutilities.linear_algebra import vec3, look_at, clip, dot, axis_to_rotation

emblem_shape = convert_img_to_shape("ChoreographyHive/assets/dacia-emblem.png")
logo_shape = convert_img_to_shape("ChoreographyHive/assets/dacia-logo.png")
emblem_positions = [vec3(0, (pos.x - 8) * 150, pos.y * 80 + 1000) for pos in emblem_shape]
logo_positions = [vec3((pos.y - 4.8) * 70, -(pos.x - 25) * 100, 17) for pos in logo_shape]

random.seed(42)
random_values = [random.random() for _ in range(128)]


class StartToEmblem(GroupStep):
    duration = 10.0
    target_indexes = range(len(emblem_shape))

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        result.car_states = {}

        for drone, emblem_pos in zip(drones, emblem_positions):
            start_pos = vec3(random_values[drone.id] * 6000 - 3000, emblem_pos.y * 3, 17)

            start_t = (abs(emblem_positions[drone.id].y)) * 0.5 / 150 + 0.5
            t = clip((self.time_since_start - start_t) * 0.3, 0, 1)

            emblem_pos = vec3(emblem_pos)
            emblem_pos.z += (self.time_since_start - self.duration) * 60

            drone_pos = smootherlerp(start_pos, emblem_pos, t)
            result.car_states[drone.id] = CarState(Physics(
                location=vec3_to_vector3(drone_pos),
                velocity=vec3_to_vector3(vec3())
            ))

            if t > 0.7:
                drone.reorient.target_orientation = look_at(vec3(0, 0, -1), vec3(-1, 0, 0))
                drone.reorient.step(self.dt)
                drone.controls = drone.reorient.controls
            else:
                drone.controls.yaw = 1
                drone.controls.roll = -1

        return result


class RotateEntireEmblem(GroupStep):
    duration = 10.0
    target_indexes = range(len(emblem_shape))

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        result.car_states = {}

        for drone, emblem_pos in zip(drones, emblem_positions):
            t = self.time_since_start / self.duration
            angle = smootherstep(0, 1, t) * math.pi * 2
            rot = axis_to_rotation(vec3(z=angle))
            drone_pos = dot(rot, emblem_pos)
            drone_pos.z -= (1 - self.time_since_start / self.duration) * 500

            result.car_states[drone.id] = CarState(Physics(
                location=vec3_to_vector3(drone_pos),
                velocity=vec3_to_vector3(vec3())
            ))

            up = dot(axis_to_rotation(vec3(z=angle)), vec3(-1, 0, 0))
            drone.reorient.target_orientation = look_at(vec3(0, 0, -1), up)
            drone.reorient.step(self.dt)
            drone.controls = drone.reorient.controls

        return result


class GoUp(GroupStep):
    duration = 5.0
    target_indexes = range(len(emblem_shape))

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        result.car_states = {}

        for drone, emblem_pos in zip(drones, emblem_positions):
            drone_pos = vec3(emblem_pos)
            drone_pos.z -= (1 - self.time_since_start / self.duration) * 500

            result.car_states[drone.id] = CarState(Physics(
                location=vec3_to_vector3(drone_pos),
                velocity=vec3_to_vector3(vec3())
            ), boost_amount=100)

        return result


class GoDown(GroupStep):
    duration = 5.0
    target_indexes = range(len(emblem_shape))

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        result.car_states = {}

        for drone, emblem_pos in zip(drones, emblem_positions):
            drone_pos = vec3(emblem_pos)
            drone_pos.z -= (self.time_since_start / self.duration) * 500

            result.car_states[drone.id] = CarState(Physics(
                location=vec3_to_vector3(drone_pos),
                velocity=vec3_to_vector3(vec3())
            ), boost_amount=100)

        return result


class RotateCars(GroupStep):
    duration = 20.0
    target_indexes = range(len(emblem_shape))

    def t(self, emblem_pos: vec3) -> float:
        raise NotImplementedError

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        result.car_states = {}

        for drone, emblem_pos in zip(drones, emblem_positions):
            t = self.t(emblem_pos)

            boost = False
            if t < 1.0:
                drone.reorient.target_orientation = look_at(vec3(0.05, 0.01, -1), vec3(-1, 0, 0))
            elif t < 3.0:
                drone.reorient.target_orientation = look_at(vec3(0.05, 0.01, -1), vec3(1, 0, 0))
            elif t < 5.0:
                drone.reorient.target_orientation = look_at(vec3(0.05, 0.01, -1), vec3(-1, 0, 0))
            elif t < 7.0:
                drone.reorient.target_orientation = look_at(vec3(0.05, 0.01, 1), vec3(1, 0, 0))
            elif t < 9.0:
                drone.reorient.target_orientation = look_at(vec3(0.05, 0.01, 1), vec3(1, 0, 0))
                boost = True
            elif t < 10.0:
                drone.reorient.target_orientation = look_at(vec3(0.05, 0.01, 1), vec3(1, 0, 0))
            elif t < 13.0:
                drone.reorient.target_orientation = look_at(vec3(0.05, 0.01, 1), vec3(-1, 0, 0))
            elif t < 16.0:
                drone.reorient.target_orientation = look_at(vec3(0.05, 0.01, -1), vec3(1, 0, 0))
            else:
                drone.reorient.target_orientation = look_at(vec3(0.05, 0.01, -1), vec3(-1, 0, 0))

            drone.reorient.step(self.dt)
            drone.controls = drone.reorient.controls
            drone.controls.boost = boost

        return result


class RotateCarsHorizontal(RotateCars):
    def t(self, emblem_pos: vec3) -> float:
        return self.time_since_start - abs(emblem_pos.y) / 1000


class RotateCarsVertical(RotateCars):
    def t(self, emblem_pos: vec3) -> float:
        return self.time_since_start - abs(emblem_pos.z) / 1000


class Madness(PerDroneStep):
    duration = 15.0
    target_indexes = range(len(emblem_shape))

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        drone.controls.boost = True
        drone.reorient.target_orientation = axis_to_rotation(vec3(
            math.sin(self.time_since_start),
            math.cos(self.time_since_start),
            math.cos(self.time_since_start + 1)
        ))
        drone.reorient.step(self.dt)
        drone.controls = drone.reorient.controls
        drone.controls.boost = True


cost_matrix = numpy.array([[
    distance(emblem_positions[i], logo_positions[j])
    for j in range(len(logo_positions))]
    for i in range(len(emblem_positions))]
)
_, target_logo = linear_sum_assignment(cost_matrix)
print(target_logo)


class EmblemToLogo(GroupStep):
    duration = 15.0
    target_indexes = range(len(emblem_shape))

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        result.car_states = {}

        for drone, emblem_pos in zip(drones, emblem_positions):
            start_t = emblem_shape[drone.id].y * 0.4
            t = clip((self.time_since_start - start_t) * 0.1, 0, 1)
            drone_pos = smootherlerp(emblem_pos, logo_positions[target_logo[drone.id]], t)

            if t < 0.9:
                result.car_states[drone.id] = CarState(Physics(
                    location=vec3_to_vector3(drone_pos),
                    velocity=vec3_to_vector3(vec3(z=1))
                ))

            if t > 0.7:
                drone.reorient.target_orientation = look_at(vec3(0, -1, 0), vec3(0, 0, 1))
                drone.reorient.step(self.dt)
                drone.controls = drone.reorient.controls
            else:
                drone.controls.yaw = 0.1
                drone.controls.roll = (drone.id % 3 - 1) * 0.1

        return result


class Logo(StateSettingStep):
    target_indexes = range(len(logo_positions))

    def set_drone_states(self, drones: List[Drone]):
        for drone, pos in zip(drones, logo_positions):
            drone.position = pos
            drone.orientation = look_at(vec3(0, -1, 0), vec3(0, 0, 1))
            drone.angular_velocity = vec3()
            drone.velocity = vec3()


class DaciaEmblemAirshow(Choreography):
    map_name = "ChampionsField"

    def generate_sequence(self):
        self.sequence = [
            HideBall(),
            HideDrones(),
            StartToEmblem(),
            ParallelStep([
                CompositeStep([
                    GoDown(),
                    GoUp(),
                    GoDown(),
                    GoUp(),
                    GoDown(),
                    GoUp(),
                    GoDown(),
                    GoUp(),
                    GoDown(),
                    GoUp(),
                    GoDown(),
                ]),
                CompositeStep([
                    RotateCarsHorizontal(),
                    Madness(),
                    RotateCarsVertical(),
                ]),
            ]),
            RotateEntireEmblem(),
            EmblemToLogo(),
            Logo(),
            Wait(10.0),
        ]

    @staticmethod
    def get_num_bots() -> int:
        return 90

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        return ["dacia.cfg"] * num_bots
