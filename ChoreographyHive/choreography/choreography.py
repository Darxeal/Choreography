from typing import Set, List

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.drone import Drone
from choreography.group_step import GroupStep


class Choreography:

    def __init__(self, game_interface: GameInterface):
        self.game_interface = game_interface
        self.sequence: List[GroupStep] = []
        self.sequence_index = 0
        self.finished = False
        self.last_frame_time = 0.0

    def step(self, packet: GameTickPacket, drones: List[Drone]):
        if self.sequence_index < len(self.sequence):
            step = self.sequence[self.sequence_index]

            time = packet.game_info.seconds_elapsed
            step.dt = time - self.last_frame_time
            self.last_frame_time = time

            result = step.perform(packet, drones, self.game_interface)

            self.game_interface.renderer.begin_rendering()
            step.render(self.game_interface.renderer)
            self.game_interface.renderer.end_rendering()

            if result.finished:
                self.sequence_index += 1
        else:
            self.finished = True

    def generate_sequence(self):
        pass

    @staticmethod
    def get_num_bots():
        raise NotImplementedError
