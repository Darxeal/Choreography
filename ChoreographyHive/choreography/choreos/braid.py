from typing import List

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, PerDroneStep, StateSettingStep, TwoTickStateSetStep

from rlutilities.linear_algebra import vec3

from .examples import YeetTheBallOutOfTheUniverse, Wait

class Braids(Choreography):
    map_name = "UtopiaRetro"

    @staticmethod
    def get_num_bots():
        return 64

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        appearances = ['default.cfg'] * num_bots
        return appearances

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        teams = [0] * num_bots
        return teams

    @staticmethod
    def get_names(num_bots: int) -> List[str]:
        names = [str(i) for i in range(num_bots)]
        return names

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            Wait(1.0),
            Centre(),
        ]


CENTRE = vec3(-15000, -15000, 0)


class Centre(TwoTickStateSetStep):
    height = 50

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            drone.position = CENTRE
            drone.position[2] += i * self.height
            drone.velocity = vec3(0, 0, 0)
            drone.angular_velocity = vec3(0, 0, 0)
