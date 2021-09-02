from typing import List

from rlbot.utils.structures.game_data_struct import GameTickPacket

from choreography.drone import Drone
from choreography.steps.base_steps import DroneListStep, BlindBehaviorStep
from rlutilities.simulation import Input


class WaitKickoffCountdown(DroneListStep):
    def step(self, packet: GameTickPacket, drones: List[Drone]):
        if packet.game_info.is_round_active:
            self.finished = True


class Wait(BlindBehaviorStep):
    def __init__(self, how_long: float = 1.0):
        self.duration = how_long
        super().__init__()

    def set_controls(self, controls: Input):
        pass
