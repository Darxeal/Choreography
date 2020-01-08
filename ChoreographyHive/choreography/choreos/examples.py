import math
from typing import List

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.drone import Drone
from choreography.group_step import BlindBehaviorStep, DroneListStep, StepResult, PerDroneStep, StateSettingStep

from rlutilities.linear_algebra import vec3, rotation, dot, vec2, look_at, normalize, xy, angle_between, \
    axis_to_rotation
from rlutilities.simulation import Ball, Input


# PerDroneStep is the base class you will probably be using the most
# it simply runs the 'step' method for every drone
class FlyUp(PerDroneStep):
    duration = 1.0  # set this attribute if you want to limit the time this step runs for

    # you can override the constructor if you need some persistent variables
    def __init__(self):
        super().__init__()

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        # you have access to these class attributes
        self.time  # the current game time
        self.start_time  # game time when this step started
        self.time_since_start  # for how long this step has been running
        self.dt  # time since last frame
        self.finished = False  # set to True if you want to finish this step early

        # lets make the drones fly up with the Aerial mechanic
        drone.aerial.target = drone.position + vec3(0, 0, 500)
        drone.aerial.up = normalize(drone.position)
        # aerial.up is where the car should orient its roof when flying
        drone.aerial.arrival_time = self.time + 0.5  # some time in the near future so it flies fast
        drone.aerial.step(self.dt)
        drone.controls = drone.aerial.controls
        drone.controls.jump = True  # you can modify the controls


# the same thing, but implemented with AerialTurn
class FlyUpAerialTurn(PerDroneStep):
    duration = 1.0

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        up = normalize(drone.position)
        drone.aerial_turn.target = look_at(vec3(0, 0, 1), up)
        drone.aerial_turn.step(self.dt)
        drone.controls = drone.aerial_turn.controls
        drone.controls.boost = angle_between(vec3(0, 0, 1), drone.forward()) < 0.5
        drone.controls.jump = True


# or you can use DroneListStep, if you want to loop over the drones yourself
class DroneListStepExample(DroneListStep):
    def step(self, packet: GameTickPacket, drones: List[Drone]):
        pass


# in StateSettingStep, all you have to do is override
# 'set_ball_state' or 'set_drone_states' or both, and set
# the fields you want to change
# btw this class is just for one-time state setting, not continuous
class YeetTheBallOutOfTheUniverse(StateSettingStep):
    def set_ball_state(self, ball: Ball):
        ball.position = vec3(0, 0, 3000)
        ball.velocity = vec3(0, 0, 0)
        ball.angular_velocity = vec3(0, 0, 0)


class FormACircle(StateSettingStep):
    radius = 2000
    center = vec3(0, 0, 0)

    def set_drone_states(self, drones: List[Drone]):
        for i, drone in enumerate(drones):
            angle = i * math.pi * 2 / len(drones)
            rot = rotation(angle)
            v = vec3(dot(rot, vec2(1, 0)))
            drone.position = v * self.radius + self.center
            drone.orientation = look_at(v * -1, vec3(0, 0, 1))
            drone.velocity = vec3(0, 0, 0)


class HoverExample(PerDroneStep):
    duration = 10.0

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        drone.hover.up = normalize(drone.position)
        clockwise_rotation = axis_to_rotation(vec3(0, 0, 0.5))
        position_on_circle = normalize(xy(drone.position)) * 2000
        drone.hover.target = dot(clockwise_rotation, position_on_circle)
        drone.hover.target[2] = 1000
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls


# BlindBehaviorStep just blindly sets controls to all bots
class DriveForward(BlindBehaviorStep):
    duration = 0.1

    def set_controls(self, controls: Input):
        controls.throttle = 1


class Wait(BlindBehaviorStep):
    duration = 1.0

    def set_controls(self, controls: Input):
        pass


class ExampleChoreography(Choreography):

    @staticmethod
    def get_num_bots():
        return 32

    def __init__(self, game_interface: GameInterface):
        super().__init__(game_interface)

    def generate_sequence(self):
        self.sequence = [
            YeetTheBallOutOfTheUniverse(),
            FormACircle(),
            Wait(),
            FlyUp(),
            HoverExample()
        ]
