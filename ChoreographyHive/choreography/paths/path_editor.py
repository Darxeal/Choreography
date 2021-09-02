import functools
import itertools
import math
import time
from ctypes import windll, create_unicode_buffer
from typing import Optional, List

import keyboard
import numpy as np
from rlbot.gateway_util import NetworkingRole
from rlbot.matchconfig.loadout_config import LoadoutConfig
from rlbot.matchconfig.match_config import MutatorConfig, MatchConfig, PlayerConfig
from rlbot.messages.flat.Color import Color
from rlbot.setup_manager import SetupManager
from rlbot.utils.rendering.rendering_manager import RenderingManager
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface

from choreography.drone import Drone
from choreography.paths.path_followers import FormationPathFollower
from choreography.steps.base_steps import GroupStep, StepResult
from choreography.paths.path import SplinePath, get_orientation_up, OrientationSplinePath, ParameterSplinePath
from choreography.paths.path_management import FormationPath
from choreography.steps.higher_order import ParallelStep
from rlutilities.linear_algebra import vec3, cross, axis_to_rotation, dot, normalize

POINT_CHANGE_SPEED = 1000.0  # uu per second
ANGLE_CHANGE_SPEED = 2.0  # radians per second
PARAM_CHANGE_SPEED = 1.0  # value per second


def get_focused_window_title() -> Optional[str]:
    # https://stackoverflow.com/questions/10266281/obtain-active-window-using-python
    hWnd = windll.user32.GetForegroundWindow()
    length = windll.user32.GetWindowTextLengthW(hWnd)
    buf = create_unicode_buffer(length + 1)
    windll.user32.GetWindowTextW(hWnd, buf, length + 1)
    return buf.value or None


class ListGetter:
    def __init__(self, array: list, index: int):
        self.array = array
        self.index = index

    def __getitem__(self, key):
        return self.array[key][self.index]

    def __setitem__(self, key, value):
        self.array[key][self.index] = value


class PathEditor:
    def __init__(self, renderer: RenderingManager, paths: List[FormationPath]):
        self.renderer = renderer

        self.paths = paths
        self.paths_iter = itertools.cycle(paths)
        self.path = next(self.paths_iter)

        self.selected_index = 0
        self.selected_list_index = 0
        self.need_to_redraw = True

        self.change_values = [POINT_CHANGE_SPEED] * 3 + [ANGLE_CHANGE_SPEED] * 2 + [PARAM_CHANGE_SPEED] * 999

    @property
    def params(self):
        hardcoded_params = [
            ListGetter(self.path.points, 0),
            ListGetter(self.path.points, 1),
            ListGetter(self.path.points, 2),
            self.path.formation_orientation,
            self.path.drone_orientation,
        ]
        return hardcoded_params + self.path.params

    def handle_keypress(self, dt: float):
        title = get_focused_window_title()
        if not title or not title.startswith("Rocket League"):
            return

        # select previous
        if keyboard.is_pressed("down arrow") and self.selected_index > 0:
            self.selected_index -= 1
            time.sleep(0.1)

        # select next
        elif keyboard.is_pressed("up arrow") and self.selected_index < len(self.path.points) - 1:
            self.selected_index += 1
            time.sleep(0.1)

        # move selected
        elif keyboard.is_pressed("left arrow"):
            self.params[self.selected_list_index][self.selected_index] -= \
                self.change_values[self.selected_list_index] * dt

        elif keyboard.is_pressed("right arrow"):
            self.params[self.selected_list_index][self.selected_index] += \
                self.change_values[self.selected_list_index] * dt

        # reset angle or parameter to 0
        elif keyboard.is_pressed("0") and self.selected_list_index > 2:
            self.params[self.selected_list_index][self.selected_index] = 0

        # set parameter to 1 or angle to pi (upside down)
        elif keyboard.is_pressed("1"):
            if self.selected_list_index in [3, 4]:
                self.params[self.selected_list_index][self.selected_index] = math.pi
            elif self.selected_list_index > 4:
                self.params[self.selected_list_index][self.selected_index] = 1

        # switch mode
        elif keyboard.is_pressed("+"):
            if self.selected_list_index < len(self.params) - 1:
                self.selected_list_index += 1
                self.renderer.clear_all_touched_render_groups()
                time.sleep(0.2)

        elif keyboard.is_pressed("-"):
            if self.selected_list_index > 0:
                self.selected_list_index -= 1
                self.renderer.clear_all_touched_render_groups()
                time.sleep(0.2)

        # insert new one after selected
        elif keyboard.is_pressed("*"):
            if self.selected_index == len(self.path.points) - 1:
                new_point = 2 * self.path.points[-1] - self.path.points[-2]
            else:
                new_point = (self.path.points[self.selected_index] + self.path.points[
                    self.selected_index + 1]) * 0.5

            self.selected_index += 1

            self.path.points.insert(self.selected_index, new_point)
            self.path.drone_orientation.insert(self.selected_index, 0)
            self.path.formation_orientation.insert(self.selected_index, 0)
            for param in self.path.params:
                param.insert(self.selected_index, 0)

            time.sleep(0.2)

        # delete selected
        elif keyboard.is_pressed("delete"):
            del self.path.points[self.selected_index]
            del self.path.drone_orientation[self.selected_index]
            del self.path.formation_orientation[self.selected_index]
            for param in self.path.params:
                del param[self.selected_index]

            self.selected_index -= 1
            time.sleep(0.2)

        # switch path
        elif keyboard.is_pressed("9"):
            self.path = next(self.paths_iter)
            time.sleep(0.2)

        # save paths
        elif keyboard.is_pressed("enter"):
            for path in self.paths:
                path.save()
            self.renderer.clear_all_touched_render_groups()
            time.sleep(0.2)
            raise SystemExit

        self.selected_list_index = max(min(self.selected_list_index, len(self.params) - 1), 0)
        self.selected_index = max(min(self.selected_index, len(self.path.points) - 1), 0)

    def draw_path(self, path: SplinePath):
        self.renderer.begin_rendering("path")
        self.renderer.draw_polyline_3d(path.curve.points, self.renderer.blue())
        self.renderer.end_rendering()

        # for i in range(30):
        #     pos = curve.point_at((1 - i / 30) * curve.length)
        #     self.renderer.draw_string_3d(pos, 1, 1, str(i), self.renderer.white())

        self.renderer.begin_rendering("points")
        for i, point in enumerate(self.path.points):
            color = self.renderer.yellow() if i == self.selected_index else self.renderer.red()
            self.renderer.draw_rect_3d(point, 4, 4, True, color, True)
        self.renderer.end_rendering()

    def draw_orientation_path(self, orientation_path: OrientationSplinePath, color: Color):
        selected_t = orientation_path.path.find_nearest(self.path.points[self.selected_index])

        self.renderer.begin_rendering(str(color))
        polyline = []
        for t in np.linspace(max(selected_t - 5000, 0), min(selected_t + 5000, orientation_path.path.length), 100):
            p = orientation_path.path.point_at(t)
            pu = p + orientation_path.up_at(t) * 200
            self.renderer.draw_line_3d(p, pu, color)
            polyline.append(pu)
        self.renderer.end_rendering()

        self.renderer.begin_rendering(str(color) + "-poly")
        self.renderer.draw_polyline_3d(polyline, color)
        self.renderer.end_rendering()

    def draw_param_visualization(self, param_path: ParameterSplinePath, orientation_path: OrientationSplinePath):
        polyline = []
        self.renderer.begin_rendering("param")
        for t in np.linspace(0, param_path.path.length, 100):
            p = param_path.path.point_at(t)
            value = param_path.value_at(t)
            pu = p + orientation_path.up_at(t) * 200 * value
            self.renderer.draw_line_3d(p, pu, self.renderer.black())
            self.renderer.draw_string_3d(pu, 1, 1, f"{value:0.1f}", self.renderer.black())
            polyline.append(pu)
        self.renderer.end_rendering()

        self.renderer.begin_rendering("param-poly")
        self.renderer.draw_polyline_3d(polyline, self.renderer.black())
        self.renderer.end_rendering()

    def draw_circle(self, pos: vec3, radius: float, color: Color, up: vec3 = None, segments=20):
        step = 2 * math.pi / segments
        points = []
        left = normalize(cross(up)) if up else vec3(0, 1, 0)
        up = up if up else vec3(0, 0, 1)

        for i in range(segments):
            angle = step * i
            p = dot(axis_to_rotation(angle * up), left) * radius
            points.append(p + pos)

        self.renderer.draw_polyline_3d(points + [points[0]], color)

    def draw_tooltip(self, path: SplinePath):
        self.renderer.begin_rendering("tooltip")
        color = self.renderer.black()
        point = self.path.points[self.selected_index]

        if self.selected_list_index < 3:
            offset = vec3(0, 0, 0)
            offset[self.selected_list_index] = 1
            self.renderer.draw_line_3d(point + offset * 200, point - offset * 200, color)
            self.renderer.draw_string_3d(point - offset * 300, 1, 1, "left", color)
            self.renderer.draw_string_3d(point + offset * 300, 1, 1, "right", color)

        elif self.selected_list_index in [3, 4]:
            angle = self.params[self.selected_list_index][self.selected_index]
            tangent = path.tangent_at(path.find_nearest(point))
            up = get_orientation_up(tangent, angle)
            self.renderer.draw_line_3d(point, point + up * 200, self.renderer.green())
            self.renderer.end_rendering()
            self.renderer.begin_rendering("circle")
            self.draw_circle(point, 200, self.renderer.green(), tangent, segments=10)

        self.renderer.end_rendering()

    def draw_other_paths(self):
        self.renderer.begin_rendering("other paths")
        for path in self.paths:
            if path is not self.path:
                spline_path = SplinePath(path.points, n_points=100)
                self.renderer.draw_polyline_3d(spline_path.curve.points, self.renderer.white())
        self.renderer.end_rendering()

    def visualize(self):
        spline_path = SplinePath(self.path.points, n_points=380)
        drone_orientation = OrientationSplinePath(spline_path, self.path.drone_orientation)
        formation_orientation = OrientationSplinePath(spline_path, self.path.formation_orientation)

        self.draw_path(spline_path)
        self.draw_other_paths()

        if self.selected_list_index in [3, 4]:
            color = self.renderer.cyan() if self.selected_list_index == 3 else self.renderer.blue()
            self.draw_orientation_path(formation_orientation, color)
            color = self.renderer.pink() if self.selected_list_index == 4 else self.renderer.purple()
            self.draw_orientation_path(drone_orientation, color)

        if self.selected_list_index > 4:
            param_path = ParameterSplinePath(spline_path, self.params[self.selected_list_index])
            self.draw_param_visualization(param_path, formation_orientation)

        self.draw_tooltip(spline_path)

    def step(self, dt: float):
        self.handle_keypress(dt)
        self.visualize()

    def loop(self):
        dt = 0.02
        while True:
            self.step(dt)
            time.sleep(dt)


class PathEditorStep(ParallelStep):
    def __init__(self, paths: List[FormationPath], followers: List[FormationPathFollower] = []):
        super().__init__(steps=followers)
        self.paths = paths
        self.editor = PathEditor(renderer=None, paths=paths)
        self.followers = followers

    def perform(self, packet: GameTickPacket, drones: List[Drone], interface: GameInterface) -> StepResult:
        t = SplinePath(self.editor.path.points).find_nearest(self.editor.path.points[self.editor.selected_index])

        for follower in self.followers:
            follower.boost = False
            if follower.path is self.editor.path:
                follower.recalculate_path(n_points=400)
            follower.set_drone_states = functools.partial(follower.state_set_on_path, formation_t=t, speed=0.1)

        result = super().perform(packet, drones, interface)
        result.finished = self.finished
        return result

    def render(self, renderer: RenderingManager):
        self.editor.renderer = renderer
        try:
            self.editor.step(self.dt)
        except SystemExit:
            self.finished = True


if __name__ == "__main__":
    sm = SetupManager()
    interface = sm.game_interface
    sm.connect_to_game()

    match_config = MatchConfig()
    match_config.game_mode = 'Soccer'
    match_config.game_map = 'UtopiaRetro'
    match_config.existing_match_behavior = 'Continue And Spawn'
    match_config.mutators = MutatorConfig()
    match_config.mutators.match_length = 'Unlimited'
    match_config.skip_replays = False
    match_config.instant_start = False
    match_config.enable_lockstep = False
    match_config.networking_role = NetworkingRole.none
    match_config.enable_rendering = True
    match_config.enable_state_setting = True
    match_config.auto_save_replay = False

    # there needs to be at least one bot for the match to start
    bot_config = PlayerConfig()
    bot_config.name = "Dummy"
    bot_config.team = 0
    bot_config.bot = True
    bot_config.rlbot_controlled = True
    bot_config.loadout_config = LoadoutConfig()
    match_config.player_configs = [bot_config]

    sm.load_match_config(match_config)
    sm.start_match()

    editor = PathEditor(interface.renderer, [FormationPath.load("air_force.json")])
    editor.loop()
