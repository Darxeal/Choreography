from abc import ABC
from typing import List

from choreography.choreography import Choreography
from choreography.choreos.air_race.formations import PhoenixCore, PurpleDragon, \
    ShapeShifter, AirForceCore, AirForceExtension, PhoenixExtension, ShapeShifterExtension, pentagon, circle, \
    BlueDragon, AirForceSmall, line, triangle
from choreography.choreos.air_race.groups import all_groups
from choreography.drone import Drone
from choreography.paths.path_editor import PathEditorStep
from choreography.paths.path_followers import SpeedControlPoint
from choreography.paths.path_management import FormationPath
from choreography.steps.base_steps import StateSettingStep
from choreography.steps.higher_order import ParallelStep, DelayedStep, CompositeStep
from choreography.steps.utils import WaitKickoffCountdown, Wait
from rlutilities.linear_algebra import vec3


class StartPaths:
    phoenix = FormationPath.load("start/phoenix.json")
    purple_dragon = FormationPath.load("start/purple-dragon.json")
    shape_shifter = FormationPath.load("start/shape-shifter.json")
    air_force = FormationPath.load("start/air-force.json")


class JoiningPaths:
    air_force_core = FormationPath.load("joining/air-force-core.json")
    air_force_ext = FormationPath.load("joining/air-force-ext.json")

    phoenix_core = FormationPath.load("joining/phoenix-core.json")
    phoenix_ext = FormationPath.load("joining/phoenix-ext.json")

    purple_dragon = FormationPath.load("joining/purple-dragon.json")
    blue_dragon = FormationPath.load("joining/blue-dragon.json")

    shape_shifter_core = FormationPath.load("joining/shape-shifter-core.json")
    shape_shifter_ext = FormationPath.load("joining/shape-shifter-ext.json")


class AquadomePaths:
    phoenix = FormationPath.load("aquadome/phoenix.json")
    purple_dragon = FormationPath.load("aquadome/purple-dragon.json")
    blue_dragon = FormationPath.load("aquadome/blue-dragon.json")
    shape_shifter = FormationPath.load("aquadome/shape-shifter.json")
    air_force = FormationPath.load("aquadome/air-force.json")


class TeleportAllToSafety(StateSettingStep):
    target_indexes = all_groups.indexes

    def set_drone_states(self, drones: List[Drone]):
        for drone in drones:
            drone.position = vec3(-8671.326171875, -40310.0, 665.0 + drone.id * 100)


class AirRace(Choreography, ABC):
    map_name = "AquaDome"

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        return all_groups.appearances

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        return all_groups.teams

    @staticmethod
    def get_num_bots():
        return len(all_groups.indexes)

    def generate_sequence(self):
        raise NotImplementedError


class AirRaceStart(AirRace):
    map_name = "UtopiaRetro"  # Custom map - Panic's Air Race

    def generate_sequence(self):
        teleport_to_start = ParallelStep([
            AirForceSmall(StartPaths.air_force, speed_plan=100, boost=False),
            PhoenixCore(StartPaths.phoenix, speed_plan=100, boost=False),
            PurpleDragon(StartPaths.purple_dragon, speed_plan=100, boost=False),
            ShapeShifter(StartPaths.shape_shifter, [line], speed_plan=100, boost=False),
            Wait(0.1),  # This is just a hack to make this step end after 0.1 seconds, enough time for a teleport
        ])

        speed_multiplier = 1.2

        def start_speed_plan(max_speed):
            return [SpeedControlPoint(0.0, 0.0), SpeedControlPoint(2.5, max_speed * speed_multiplier)]

        start_to_cave = ParallelStep([
            AirForceSmall(StartPaths.air_force, start_speed_plan(2300)),
            DelayedStep(
                PhoenixCore(StartPaths.phoenix, start_speed_plan(2250)),
                1.0
            ),
            DelayedStep(
                PurpleDragon(StartPaths.purple_dragon, start_speed_plan(2300)),
                2.0
            ),
            DelayedStep(
                ShapeShifter(StartPaths.shape_shifter, [line, pentagon], speed_plan=start_speed_plan(2350)),
                2.7
            ),
        ])

        self.sequence = [
            WaitKickoffCountdown(),
            teleport_to_start,
            Wait(10.0),
            start_to_cave,
        ]


class AirRaceMiddle(AirRace):
    map_name = "UtopiaRetro"  # Custom map - Panic's Air Race

    def generate_sequence(self):
        speed_multiplier = 1.1

        def wormhole_plan(delay=0.0, multiplier=1.0):
            # return 2300 * multiplier
            return [
                SpeedControlPoint(delay + 10.0 / speed_multiplier, 2300 * multiplier * speed_multiplier),
                SpeedControlPoint(delay + 10.5 / speed_multiplier, 6000 * multiplier * speed_multiplier),
                SpeedControlPoint(delay + 15.0 / speed_multiplier, 6000 * multiplier * speed_multiplier),
                SpeedControlPoint(delay + 15.5 / speed_multiplier, 2300 * multiplier * speed_multiplier),
            ]

        air_force = [
            AirForceCore(JoiningPaths.air_force_core, speed_plan=wormhole_plan(delay=2.0)),
            CompositeStep([
                DelayedStep(
                    AirForceExtension(JoiningPaths.air_force_ext, speed_plan=2300 * speed_multiplier),
                    delay=1.57 / speed_multiplier
                ),
                AirForceExtension(JoiningPaths.air_force_core, speed_plan=wormhole_plan(delay=2.0)),
            ], shared_start_time=True),
        ]

        phoenix = [
            PhoenixCore(JoiningPaths.phoenix_core, speed_plan=wormhole_plan(4.0)),
            CompositeStep([
                PhoenixExtension(JoiningPaths.phoenix_ext, speed_plan=wormhole_plan(4.0)),
                PhoenixExtension(JoiningPaths.phoenix_core, speed_plan=wormhole_plan(4.0)),
            ], shared_start_time=True),
        ]

        shape_shifter = [
            ShapeShifter(JoiningPaths.shape_shifter_core, [circle[:5], pentagon, triangle[:5]],
                         speed_plan=wormhole_plan(6.0)),
            CompositeStep([
                ShapeShifterExtension(JoiningPaths.shape_shifter_ext, [circle[5:], pentagon, triangle[:5]],
                                      speed_plan=wormhole_plan(6.0)),
                ShapeShifterExtension(JoiningPaths.shape_shifter_core, [circle[5:], pentagon, triangle[5:]],
                                      speed_plan=wormhole_plan(6.0)),
            ], shared_start_time=True),
        ]

        dragons = [
            PurpleDragon(JoiningPaths.purple_dragon, speed_plan=wormhole_plan(multiplier=1.0)),
            BlueDragon(JoiningPaths.blue_dragon, speed_plan=wormhole_plan(multiplier=1.0)),
        ]

        self.sequence = [
            WaitKickoffCountdown(),
            ParallelStep(air_force + phoenix + shape_shifter + dragons, finish_when=all),
        ]


rectangle = [
    vec3(0, -300, 200),
    vec3(0, -100, 200),
    vec3(0, 100, 200),
    vec3(0, 300, 200),
    vec3(0, 300, 0),
    vec3(0, 300, -200),
    vec3(0, 100, -200),
    vec3(0, -100, -200),
    vec3(0, -300, -200),
    vec3(0, -300, 0),
]


class AirRaceAquadome(AirRace):
    map_name = "AquaDome"

    def generate_sequence(self):
        aquadome_speed = 2800

        ss_pentagon_ext = [o * 1.5 for o in pentagon]
        ss_pentagon_ext[0], ss_pentagon_ext[2] = ss_pentagon_ext[2], ss_pentagon_ext[0]
        ss_core = [[vec3(-200, 0, 0)] * 5, pentagon, triangle[:5], rectangle[:5]]
        ss_ext = [[vec3(200, 0, 0)] * 5, ss_pentagon_ext, triangle[5:], rectangle[5:]]
        ss_speed = [
            SpeedControlPoint(13, 2900),
            SpeedControlPoint(13.1, 4200),
            SpeedControlPoint(16.0, 4200),
            SpeedControlPoint(17.0, 3000),
        ]

        aquadome = ParallelStep([
            AirForceCore(AquadomePaths.air_force, speed_plan=2850),
            AirForceExtension(AquadomePaths.air_force, speed_plan=2850),
            PhoenixCore(AquadomePaths.phoenix, speed_plan=3000),
            PhoenixExtension(AquadomePaths.phoenix, speed_plan=3000),
            ShapeShifter(AquadomePaths.shape_shifter, shapes=ss_core, speed_plan=ss_speed),
            ShapeShifterExtension(AquadomePaths.shape_shifter, shapes=ss_ext, speed_plan=ss_speed),
            PurpleDragon(AquadomePaths.purple_dragon, speed_plan=2900),
            BlueDragon(AquadomePaths.blue_dragon, speed_plan=2900),
        ], finish_when=all)

        path_editor = PathEditorStep(
            paths=[
                AquadomePaths.air_force,
                AquadomePaths.phoenix,
                AquadomePaths.shape_shifter,
                AquadomePaths.blue_dragon,
                AquadomePaths.purple_dragon,
            ],
            followers=[
                PhoenixCore(AquadomePaths.phoenix),
                PhoenixExtension(AquadomePaths.phoenix),
                PurpleDragon(AquadomePaths.purple_dragon),
                BlueDragon(AquadomePaths.blue_dragon),
                AirForceCore(AquadomePaths.air_force),
                AirForceExtension(AquadomePaths.air_force),
                ShapeShifter(AquadomePaths.shape_shifter, shapes=ss_core),
                ShapeShifterExtension(AquadomePaths.shape_shifter, shapes=ss_ext)
            ]
        )

        self.sequence = [
            WaitKickoffCountdown(),
            aquadome,
            path_editor,
        ]
