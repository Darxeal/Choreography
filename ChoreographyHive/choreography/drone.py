from rlbot.utils.structures.game_data_struct import Rotator, Vector3, PlayerInfo

from rlutilities.simulation import Car, Input
from rlutilities.linear_algebra import vec3, mat3, euler_to_rotation

from rlutilities.mechanics import Aerial, AerialTurn


class Drone(Car):

    def __init__(self, index: int, team: int):
        super().__init__()
        self.team = team
        self.id = index
        self.aerial_turn = AerialTurn(self)
        self.aerial = Aerial(self)

    def update(self, game_car: PlayerInfo):
        self.position = vector3_to_vec3(game_car.physics.location)
        self.velocity = vector3_to_vec3(game_car.physics.velocity)
        self.orientation = rotator_to_mat3(game_car.physics.rotation)
        self.angular_velocity = vector3_to_vec3(game_car.physics.angular_velocity)
        self.boost = game_car.boost

        # Reset ctrl every tick.
        self.controls = Input()


def vector3_to_vec3(v: Vector3) -> vec3:
    return vec3(v.x, v.y, v.z)


def rotator_to_mat3(r: Rotator) -> mat3:
    return euler_to_rotation(vec3(r.pitch, r.yaw, r.roll))
