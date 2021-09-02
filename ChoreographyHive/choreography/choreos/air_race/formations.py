import math
import operator
from functools import reduce
from typing import List

from choreography.choreos.air_race.groups import purple_dragon, blue_dragon, air_force, shape_shifter, phoenix, \
    shape_shifter_ext
from choreography.paths.path_followers import FormationPathFollower
from choreography.paths.path_management import FormationPath
from rlutilities.linear_algebra import vec3

air_force_offsets = [
    vec3(0, -400, 0),
    vec3(400, -300, 0),
    vec3(800, -200, 0),
    vec3(1200, -100, 0),
    vec3(1600, 0, 0),
    vec3(1200, 100, 0),
    vec3(800, 200, 0),
    vec3(400, 300, 0),
    vec3(0, 400, 0),
]


class AirForce(FormationPathFollower):
    offset_indices: List[int] = None

    def get_formation_offsets(self, t: float) -> List[vec3]:
        offsets = [vec3(air_force_offsets[i]) for i in self.offset_indices]
        for offset in offsets:
            offset[1] *= (1 + self.params[0].value_at(t))
        return offsets


class AirForceSmall(AirForce):
    offset_indices = [2, 3, 4, 5, 6]
    target_indexes = [air_force.indexes[i] for i in [0, 1, 4, 6, 7]]


class AirForceCore(AirForce):
    offset_indices = [1, 2, 4, 6, 7]
    target_indexes = [air_force.indexes[i] for i in offset_indices]


class AirForceExtension(AirForce):
    offset_indices = [0, 3, 5, 8]
    target_indexes = [air_force.indexes[i] for i in offset_indices]


class PhoenixCore(FormationPathFollower):
    target_indexes = phoenix.indexes[:-4]

    def get_formation_offsets(self, t: float) -> List[vec3]:
        head = [
            vec3(700, 0, 0),
        ]
        wings = [
            # vec3(500, -300, 120),
            vec3(530, -200, 100),
            vec3(520, -100, 50),
            vec3(500, 0, 0),
            vec3(520, 100, 50),
            vec3(530, 200, 100),
            # vec3(500, 300, 120),
        ]
        tail = [
            vec3(0, -50, 0),
            vec3(200, 0, 0),
            vec3(0, 50, 0),
        ]

        flap_z = math.sin(t / 800) * self.params[0].value_at(t) * 2.0
        for offset in wings:
            offset[2] *= flap_z

        return head + wings + tail


# TODO: refactor
class PhoenixExtension(FormationPathFollower):
    target_indexes = phoenix.indexes[-4:]

    def get_formation_offsets(self, t: float) -> List[vec3]:
        wings = [
            vec3(510, -400, 180),
            vec3(520, -300, 150),
            vec3(520, 300, 150),
            vec3(510, 400, 180),
        ]
        flap_z = math.sin(t / 800) * self.params[0].value_at(t) * 2.0
        for offset in wings:
            offset[2] *= flap_z
        return wings


class DuelingDragon(FormationPathFollower):
    def get_formation_offsets(self, t: float) -> List[vec3]:
        return [vec3(i * 300, 0, 0) for i in range(len(self.target_indexes))]


class PurpleDragon(DuelingDragon):
    target_indexes = purple_dragon.indexes


class BlueDragon(DuelingDragon):
    target_indexes = blue_dragon.indexes


class ShapeShifter(FormationPathFollower):
    target_indexes = shape_shifter.indexes

    def __init__(self, path: FormationPath, shapes: List[List[vec3]], **kwargs):
        super().__init__(path, **kwargs)
        self.shapes = shapes

    def get_formation_offsets(self, t: float) -> List[vec3]:
        return [reduce(operator.add, [
            self.params[j].value_at(t) * self.shapes[j][i] for j in range(len(self.shapes))
        ]) for i in range(len(self.target_indexes))]


class ShapeShifterExtension(ShapeShifter):
    target_indexes = shape_shifter_ext.indexes


line = [
    vec3(0, -300, 0),
    vec3(0, -150, 0),
    vec3(0, 0, 0),
    vec3(0, 150, 0),
    vec3(0, 300, 0),
]
pentagon = [
    vec3(0, -300, 70),
    vec3(0, -170, -300),
    vec3(0, 0, 300),
    vec3(0, 170, -300),
    vec3(0, 300, 70),
]
radius = 300
circle = [
    vec3(0, 0.00, 1.00) * radius,
    vec3(0, 0.59, 0.81) * radius,
    vec3(0, 0.95, 0.31) * radius,
    vec3(0, 0.95, -0.31) * radius,
    vec3(0, 0.59, -0.81) * radius,
    vec3(0, 0.00, -1.00) * radius,
    vec3(0, -0.59, -0.81) * radius,
    vec3(0, -0.95, -0.31) * radius,
    vec3(0, -0.95, 0.31) * radius,
    vec3(0, -0.59, 0.81) * radius,
]
triangle = [
    vec3(0, 0, 300),

    vec3(0, 100, 100),
    vec3(0, 200, -100),
    vec3(0, 300, -300),

    vec3(0, 150, -300),
    vec3(0, 0, -300),
    vec3(0, -150, -300),

    vec3(0, -300, -300),
    vec3(0, -200, -100),
    vec3(0, -100, 100),
]
