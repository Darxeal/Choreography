from typing import List

from rlbot.utils.game_state_util import GameState, CarState, Physics, Vector3, Rotator, BallState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.drone import Drone, vector3_to_vec3
from rlutilities.linear_algebra import vec3, mat3, rotation_to_euler
from rlutilities.simulation import Input, Ball


class StepResult:
    def __init__(self, finished: bool = False):
        self.finished = finished


class GroupStepDuration:
    duration: float = 0.0

    def __init__(self):
        self.start_time: float = 0
        assert self.duration > 0.0

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        time = packet.game_info.seconds_elapsed
        if not self.start_time:
            self.start_time = time
        return StepResult(finished=time > self.start_time + self.duration)


class DroneListStep(GroupStepDuration):
    """
    'step' receives the entire drone list. More powerful but less
    convenient than PerDroneStep. It should be possible to accomplish almost anything
    with this one.
    """

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        self.step(packet, drones)
        return super().perform(packet, drones, interface)

    def step(self, packet: GameTickPacket, drones: List[Drone]):
        raise NotImplementedError


class PerDroneStep(GroupStepDuration):
    """
    Calls 'step' for every drone individually. They can still behave differently
    because you have access to the drone's index, position, velocity, etc.
    """

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        for drone in drones:
            self.step(packet, drone)
        return super().perform(packet, drones, interface)

    def step(self, packet: GameTickPacket, drone: Drone):
        raise NotImplementedError


class BlindBehaviorStep(PerDroneStep):
    """
    For every drone in the list, output the given controls for the specified duration.
    For example you could make everyone to boost simultaneously for .5 seconds.
    """

    def step(self, packet: GameTickPacket, drone: Drone):
        self.set_controls(drone.controls)

    def set_controls(self, controls: Input):
        raise NotImplementedError


def vec3_to_vector3(vec: vec3) -> Vector3:
    return Vector3(vec[0], vec[1], vec[2])


def mat3_to_rotator(mat: mat3) -> Rotator:
    pyr = rotation_to_euler(mat)
    return Rotator(pyr[0], pyr[1], pyr[2])


class StateSettingStep(GroupStepDuration):
    duration = 0.1  # wait a bit for state setting to take effect

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        ball = Ball()
        ball.position = vector3_to_vec3(packet.game_ball.physics.location)
        ball.velocity = vector3_to_vec3(packet.game_ball.physics.velocity)
        ball.angular_velocity = vector3_to_vec3(packet.game_ball.physics.angular_velocity)

        self.set_ball_state(ball)
        self.set_drone_states(drones)

        state = GameState(cars={})
        state.ball = BallState(
            physics=Physics(
                location=vec3_to_vector3(ball.position),
                velocity=vec3_to_vector3(ball.velocity),
                angular_velocity=vec3_to_vector3(ball.angular_velocity)
            )
        )
        for drone in drones:
            state.cars[drone.id] = CarState(
                Physics(
                    location=vec3_to_vector3(drone.position),
                    velocity=vec3_to_vector3(drone.velocity),
                    angular_velocity=vec3_to_vector3(drone.angular_velocity),
                    rotation=mat3_to_rotator(drone.orientation)
                ),
                drone.boost
            )
        interface.set_game_state(state)
        return super().perform(packet, drones, interface)

    def set_ball_state(self, ball: Ball):
        pass

    def set_drone_states(self, drones: List[Drone]):
        pass
