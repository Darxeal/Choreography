import math
from dataclasses import dataclass
from typing import List, Optional, Union

from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.drone import Drone
from choreography.steps.base_steps import PerDroneStep, StateSettingStep, GroupStep, StepResult
from choreography.paths.path import SplinePath, OrientationSplinePath, ParameterSplinePath
from choreography.paths.path_management import FormationPath
from choreography.utils.vector_math import lerp, direction
from rlutilities.linear_algebra import vec3, axis_to_rotation, dot, normalize, cross, look_at, clip
from rlutilities.simulation import Curve


class HoverPathFollower(PerDroneStep):
    speed = 500
    path: Curve = None

    def step(self, packet: GameTickPacket, drone: Drone, index: int):
        t = self.path.find_nearest(drone.position)
        target_t = t - self.speed
        if target_t < 0:
            self.finished = True
            return

        drone.hover.target = self.path.point_at(target_t)
        drone.hover.up = dot(axis_to_rotation(vec3(0, 0, self.time_since_start * 5.0)), vec3(1, 0, 0))
        drone.hover.step(self.dt)
        drone.controls = drone.hover.controls


@dataclass
class SpeedControlPoint:
    time: float
    speed: float


def displacement_with_max_speed(initial_speed, acceleration, max_speed, time) -> float:
    diff = max_speed - initial_speed
    acc_time = min(diff / acceleration, time)
    const_time = time - acc_time
    return initial_speed * acc_time + 0.5 * acceleration * acc_time ** 2 + const_time * max_speed


class FormationPathFollower(StateSettingStep):
    lookahead = 0.3
    duration = math.inf

    def __init__(self, path: FormationPath, speed_plan: Union[List[SpeedControlPoint], float] = 2300, boost=True):
        super().__init__()
        self.path = path
        self.boost = boost

        self.recalculate_path()

        if isinstance(speed_plan, (int, float)):  # constant speed
            self.speed_plan = [SpeedControlPoint(0.0, speed_plan), SpeedControlPoint(math.inf, speed_plan)]
        elif isinstance(speed_plan, list):
            self.speed_plan = speed_plan.copy()
            self.speed_plan.insert(0, SpeedControlPoint(0.0, speed_plan[0].speed))
            self.speed_plan.append(SpeedControlPoint(math.inf, speed_plan[-1].speed))
        else:
            raise ValueError("Invalid speed plan")

    def recalculate_path(self, n_points=2000):
        self.spline_path = SplinePath(self.path.points, n_points=n_points)
        self.formation_orientation = OrientationSplinePath(self.spline_path, self.path.formation_orientation)
        self.drone_orientation = OrientationSplinePath(self.spline_path, self.path.drone_orientation)
        self.params = [ParameterSplinePath(self.spline_path, param) for param in self.path.params]

    def t_and_speed(self, time: float) -> (float, float):
        t = 0
        speed = 0

        for i in range(len(self.speed_plan) - 1):
            step = self.speed_plan[i]
            next_step = self.speed_plan[i + 1]

            total_step_time = next_step.time - step.time
            step_time = min(time - step.time, total_step_time)
            if step_time <= 0:
                break

            step_accel = (next_step.speed - step.speed) / total_step_time
            speed = step.speed + step_accel * step_time
            t += step.speed * step_time + 0.5 * step_accel * step_time ** 2

        return t, speed

    def get_formation_offsets(self, t: float) -> List[vec3]:
        raise NotImplementedError

    def state_set_on_path(self, drones: List[Drone], formation_t: float, speed: float):
        offsets = self.get_formation_offsets(formation_t)
        offsets_ahead = self.get_formation_offsets(formation_t + speed * self.lookahead)

        for drone, offset, offset_ahead in zip(drones, offsets, offsets_ahead):
            drone_t = formation_t + offset[0]
            t_ahead = formation_t + offset_ahead[0] + speed * self.lookahead

            # if we're not on the path, don't do any state setting
            if drone_t < 0 or drone_t >= self.spline_path.length:
                continue

            path_pos = self.spline_path.point_at(drone_t)
            path_pos_ahead = self.spline_path.point_at(t_ahead)

            path_forward = normalize(self.spline_path.tangent_at(drone_t))

            formation_up = self.formation_orientation.up_at(drone_t)
            formation_left = normalize(cross(formation_up, path_forward))

            pos = path_pos + formation_left * offset[1] + formation_up * offset[2]
            pos_ahead = path_pos_ahead + formation_left * offset_ahead[1] + formation_up * offset_ahead[2]

            drone_forward = direction(pos, pos_ahead)
            drone_up = self.drone_orientation.up_at(drone_t)
            target_orientation = look_at(drone_forward, drone_up)

            drone.position = pos
            drone.velocity = drone_forward * speed
            drone.angular_velocity = vec3(0, 0, 0)  # TODO: setting correct angular velocity could help with replays
            drone.orientation = target_orientation
            drone.controls.boost = self.boost

    def set_drone_states(self, drones: List[Drone]):
        formation_t, speed = self.t_and_speed(self.time_since_start)

        if formation_t >= self.spline_path.length:
            self.finished = True
            return

        self.state_set_on_path(drones, formation_t, speed)
