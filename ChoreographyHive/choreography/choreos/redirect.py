from typing import List

from rlbot.utils.rendering.rendering_manager import RenderingManager
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.steps.base_steps import BlindBehaviorStep, StateSettingStep, PerDroneStep
from choreography.steps.higher_order import ParallelStep, CompositeStep
from choreography.steps.utils import WaitKickoffCountdown
from rlutilities.linear_algebra import vec3, look_at
from rlutilities.simulation import Input, Ball


class Start(StateSettingStep):

    def set_drone_states(self, drones: List[Drone]):
        drone = drones[0]
        drone.position = vec3(-9500, -12800, 30)
        drone.angular_velocity = vec3(0, 0, 0)
        drone.orientation = look_at(vec3(0, 1, 0), vec3(0, 0, 1))
        drone.velocity = vec3(0, 2300, 0)


class DriveForward(BlindBehaviorStep):
    target_indexes = [0]
    duration = 4.0

    def set_controls(self, controls: Input):
        controls.throttle = 1
        controls.steer = 0
        controls.boost = 0


class TurnLeft(BlindBehaviorStep):
    target_indexes = [0]
    duration = 0.5

    def set_controls(self, controls: Input):
        controls.throttle = 1
        controls.steer = -1
        # controls.boost = 1


intercept = vec3(-2000, 3000, 1500)
aerial_time = 4.0
yeet_delay = 2.0
yeet_time = aerial_time - yeet_delay


class YeetBall(StateSettingStep):
    duration = yeet_delay

    def set_ball_state(self, ball: Ball):
        ball.position = vec3(3000, 0, 200)
        ball.velocity = (intercept - ball.position) * 1.035 / yeet_time + vec3(0, 0, 630)


class Fly(PerDroneStep):
    duration = aerial_time + 1.0

    def __init__(self):
        super().__init__()
        self.rendered = True
        self.jump_pos = None

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        drone.aerial.target = intercept
        drone.aerial.arrival_time = self.start_time + aerial_time
        drone.aerial.up = vec3(-1, -1, 1) if self.time_since_start < 2 else vec3(0, 0, -1)
        drone.aerial.step(self.dt)
        drone.controls = drone.aerial.controls
        drone.controls.jump = True

        if self.jump_pos is None:
            self.jump_pos = vec3(drone.position)
        # if self.time_since_start > 1:
        #     drone.controls.roll = 1

    def render(self, renderer: RenderingManager):
        if not self.rendered:
            self.rendered = True
            renderer.begin_rendering("persist")
            renderer.draw_rect_3d(self.jump_pos, 10, 10, True, renderer.lime(), True)
            renderer.end_rendering()

        renderer.draw_polyline_3d([
            (-4096, -5120, 50),
            (-4096, 5120, 50),
            (4096, 5120, 50),
            (4096, -5120, 50),
            (-4096, -5120, 50),
        ], renderer.yellow())

        renderer.draw_polyline_3d([
            (-4096, -5120, 2044),
            (-4096, 5120, 2044),
            (4096, 5120, 2044),
            (4096, -5120, 2044),
            (-4096, -5120, 2044),
        ], renderer.yellow())

        renderer.draw_polyline_3d([
            (-893, 5120, 50),
            (-893, 5120, 642),
            (893, 5120, 642),
            (893, 5120, 50),
            (-893, 5120, 50),
        ], renderer.red())


class Redirect(Choreography):
    map_name = "UtopiaRetro"  # custom map - Simplicity

    def generate_sequence(self):
        self.sequence = [
            WaitKickoffCountdown(),
            Start(),
            DriveForward(),
            TurnLeft(),
            ParallelStep([
                Fly(),
                CompositeStep([
                    YeetBall(),
                    Wait(),
                ]),
            ], finish_when=all)
        ]

    @staticmethod
    def get_num_bots() -> int:
        return 1

    @staticmethod
    def get_appearances(num_bots: int) -> List[str]:
        return ["blue_dragon.cfg"]

    @staticmethod
    def get_teams(num_bots: int) -> List[int]:
        return [0]
