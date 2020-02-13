from typing import List, Iterable
from math import inf

from rlbot.utils.rendering.rendering_manager import RenderingManager
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface
from rlbot.utils.game_state_util import GameState, CarState, Physics, Vector3, Rotator, BallState

from choreography.drone import Drone, vector3_to_vec3
from rlutilities.simulation import Input, Ball

from rlutilities.linear_algebra import vec3, mat3, rotation_to_euler


class StepResult:
    def __init__(self, finished: bool = False):
        self.finished = finished


class GroupStep:
    duration: float = inf
    target_indexes: Iterable[int] = range(64)

    def __init__(self):
        self.start_time: float = None
        self.finished = False
        self.time = 0.0
        self.dt = 0.0
        self.time_since_start = 0.0

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        self.time = packet.game_info.seconds_elapsed
        if not self.start_time:
            self.start_time = self.time
            print(type(self).__name__)

        self.time_since_start = self.time - self.start_time
        return StepResult(finished=self.finished or self.time > self.start_time + self.duration)

    def render(self, renderer: RenderingManager):
        pass


class ParallelStep(GroupStep):
    """
    Multiple steps run in parallel
    """

    def __init__(self, steps: List[GroupStep]):
        super().__init__()
        self.steps = steps

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        finished = False
        for step in self.steps:
            if step.perform(packet, drones, interface).finished:
                finished = True
        return StepResult(finished)

    def render(self, renderer: RenderingManager):
        for step in self.steps:
            step.render(renderer)


class CompositeStep(GroupStep):
    """
    Multiple steps run in a sequence
    """

    def __init__(self, steps: List[GroupStep]):
        super().__init__()
        self.steps = steps
        self.current_step_index = 0

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        step = self.steps[self.current_step_index]
        if step.perform(packet, drones, interface).finished:
            self.current_step_index += 1
        return StepResult(self.current_step_index == len(self.steps))


class DroneListStep(GroupStep):
    """
    'step' receives the entire drone list. More powerful but less
    convenient than PerDroneStep. It should be possible to accomplish almost anything
    with this one.
    """

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        self.step(packet, [drone for drone in drones if drone.id in self.target_indexes])
        return super().perform(packet, drones, interface)

    def step(self, packet: GameTickPacket, drones: List[Drone]):
        raise NotImplementedError


class PerDroneStep(GroupStep):
    """
    Calls 'step' for every drone individually. They can still behave differently
    because you have access to the drone's index, position, velocity, etc.
    """

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        for index, drone in enumerate(drones):
            if drone.id in self.target_indexes:
                self.step(packet, drone, index)
        return super().perform(packet, drones, interface)

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        raise NotImplementedError


class BlindBehaviorStep(PerDroneStep):
    """
    For every drone in the list, output the given controls for the specified duration.
    For example you could make everyone to boost simultaneously for .5 seconds.
    """

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        self.set_controls(drone.controls)

    def set_controls(self, controls: Input):
        raise NotImplementedError


def vec3_to_vector3(vec: vec3) -> Vector3:
    return Vector3(vec[0], vec[1], vec[2])


def mat3_to_rotator(mat: mat3) -> Rotator:
    pyr = rotation_to_euler(mat)
    return Rotator(pyr[0], pyr[1], pyr[2])


class StateSettingStep(GroupStep):
    duration = 0.1  # wait a bit for state setting to take effect

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        ball = Ball()
        ball.position = vector3_to_vec3(packet.game_ball.physics.location)
        ball.velocity = vector3_to_vec3(packet.game_ball.physics.velocity)
        ball.angular_velocity = vector3_to_vec3(packet.game_ball.physics.angular_velocity)
        target_drones = [drone for drone in drones if drone.id in self.target_indexes]

        self.set_ball_state(ball)
        self.set_drone_states(target_drones)

        state = GameState(cars={})
        state.ball = BallState(
            physics=Physics(
                location=vec3_to_vector3(ball.position),
                velocity=vec3_to_vector3(ball.velocity),
                angular_velocity=vec3_to_vector3(ball.angular_velocity)
            )
        )
        for drone in target_drones:
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


# Use this when trying to state set >48 drones.
class TwoTickStateSetStep(StateSettingStep):
    tick = 0  # Counting ticks

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        self.tick += 1
        self.finished = self.tick >= 2

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
        # State setting only half of the drones per tick
        for drone in drones[self.tick % 2::2]:
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
