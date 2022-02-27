import random
from typing import List

from rlbot.utils.game_state_util import CarState, Physics
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface
from scipy.spatial.transform import Slerp, Rotation

from choreography.choreography import Choreography
from choreography.choreos.dacia import HideDrones, HideBall
from choreography.drone import Drone
from choreography.steps.base_steps import StateSettingStep, GroupStep, StepResult, vec3_to_vector3
from choreography.steps.utils import Wait
from choreography.utils.img_to_shape import convert_img_to_shape
from choreography.utils.vector_math import smootherlerp, distance
from rlutilities.linear_algebra import vec3, look_at, clip, euler_to_rotation

emblem_shape = convert_img_to_shape("ChoreographyHive/assets/dacia-emblem.png")
logo_shape = convert_img_to_shape("ChoreographyHive/assets/dacia-logo.png")
emblem_positions = [vec3(0, (pos.x - 8) * 150, pos.y * 80 + 1000) for pos in emblem_shape]
logo_positions = [vec3(pos.y * 70, -(pos.x - 25) * 100, 17) for pos in logo_shape]


class Emblem(GroupStep):
    duration = 10.0
    target_indexes = range(len(emblem_shape))

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        result.car_states = {}

        for drone, emblem_pos in zip(drones, emblem_positions):
            start_pos = vec3(hash(drone.id * 4.89) % 4000 - 2000, emblem_pos.y - 3000, 17)

            start_t = (17 - emblem_shape[drone.id].x) * 0.2
            t = clip((self.time_since_start - start_t) * 0.3, 0, 1)

            emblem_pos = vec3(emblem_pos)
            emblem_pos.z += (self.time_since_start - self.duration) * 60

            drone_pos = smootherlerp(start_pos, emblem_pos, t)
            result.car_states[drone.id] = CarState(Physics(
                location=vec3_to_vector3(drone_pos),
                velocity=vec3_to_vector3(vec3(z=100))
            ))

            if t > 0.7:
                drone.reorient.target_orientation = look_at(vec3(0, 0, -1), vec3(1, 0, 0))
                drone.reorient.step(self.dt)
                drone.controls = drone.reorient.controls
            else:
                drone.controls.yaw = 1
                drone.controls.roll = -1

        return result


free_logo = set(logo_positions)
target_logo = dict()
indices = list(range(len(emblem_positions)))
random.shuffle(indices)
for index in indices:
    nearest_logo = min(free_logo, key=lambda pos: distance(pos, emblem_positions[index]))
    free_logo.remove(nearest_logo)
    target_logo[index] = nearest_logo


class EmblemToLogo(GroupStep):
    duration = 12.0
    target_indexes = range(len(emblem_shape))

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        result = super().perform(packet, drones, interface)
        result.car_states = {}

        for drone, emblem_pos in zip(drones, emblem_positions):
            start_t = emblem_shape[drone.id].y * 0.3
            t = clip((self.time_since_start - start_t) * 0.1, 0, 1)
            drone_pos = smootherlerp(emblem_pos, target_logo[drone.id], t)

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
            Emblem(),
            EmblemToLogo(),
            Logo(),
            Wait(5.0),
        ]

    @staticmethod
    def get_num_bots() -> int:
        return 90

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        return ["dacia.cfg"] * num_bots
