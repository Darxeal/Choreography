from typing import List, Callable

from rlbot.utils.rendering.rendering_manager import RenderingManager
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.drone import Drone
from choreography.steps.base_steps import GroupStep, StepResult


class ParallelStep(GroupStep):
    """
    Multiple steps run in parallel
    """

    def __init__(self, steps: List[GroupStep], finish_when: Callable[[List[bool]], bool] = any):
        super().__init__()
        self.steps = steps
        self.finish_when = finish_when

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        super().perform(packet, drones, interface)

        statuses = []
        car_states = {}
        ball_state = None

        for step in self.steps:
            step.dt = self.dt
            if step.finished:
                continue

            result = step.perform(packet, drones, interface)

            statuses.append(result.finished)
            if result.car_states:
                car_states.update(result.car_states)
            if result.ball_state:
                ball_state = result.ball_state

        return StepResult(finished=self.finish_when(statuses), car_states=car_states, ball_state=ball_state)

    def render(self, renderer: RenderingManager):
        for step in self.steps:
            step.render(renderer)


class CompositeStep(GroupStep):
    """
    Multiple steps run in a sequence
    """

    def __init__(self, steps: List[GroupStep], shared_start_time=False):
        super().__init__()
        self.steps = steps
        self.shared_start_time = shared_start_time
        self.current_step_index = 0

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        self.time = packet.game_info.seconds_elapsed
        if self.start_time is None:
            self.start_time = self.time
            if self.shared_start_time:
                for step in self.steps:
                    step.start_time = self.start_time

        if self.current_step_index < len(self.steps):
            step = self.steps[self.current_step_index]
            step.dt = self.dt
            result = step.perform(packet, drones, interface)

            if result.finished:
                self.current_step_index += 1
        else:
            result = StepResult(finished=True)
        return result

    def render(self, renderer: RenderingManager):
        self.steps[min(self.current_step_index, len(self.steps) - 1)].render(renderer)


class DelayedStep(GroupStep):
    """
    A step wrapper which delays the step by a specified amount of time (seconds)
    """
    def __init__(self, step: GroupStep, delay: float):
        super().__init__()
        self.step = step
        self.delay = delay

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        if self.time_since_start > self.delay:
            self.step.dt = self.dt
            return self.step.perform(packet, drones, interface)
        return super().perform(packet, drones, interface)
