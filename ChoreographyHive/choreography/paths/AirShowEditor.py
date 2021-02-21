from copy import copy
from math import pi

from rlbot.matchconfig.match_config import MutatorConfig, MatchConfig, PlayerConfig
from rlbot.setup_manager import SetupManager
from rlbot.utils.rendering.rendering_manager import RenderingManager
from rlbot.utils.structures.game_interface import GameInterface
from rlbot.utils.structures.game_data_struct import GameTickPacket
import time
import keyboard
from rlutilities.simulation import Curve

from rlutilities.linear_algebra import vec3, dot, axis_to_rotation

from AirShowPath import zero
from bezier import BezierPath


class BezierPathEditor:
    def __init__(self):
        match_config = MatchConfig()
        match_config.game_mode = 'Soccer'
        match_config.game_map = 'Mannfield_Night'
        match_config.existing_match_behavior = 'Continue And Spawn'
        match_config.mutators = MutatorConfig()
        match_config.mutators.match_length = 'Unlimited'

        # there needs to be at least one bot for the match to start
        bot_config = PlayerConfig()
        bot_config.name = "Dummy"
        bot_config.team = 0
        bot_config.bot = True
        bot_config.rlbot_controlled = True
        match_config.player_configs = [bot_config]

        sm = SetupManager()
        sm.connect_to_game()
        sm.load_match_config(match_config)
        sm.start_match()
        self.game_interface: GameInterface = sm.game_interface

    def __del__(self):
        self.game_interface.renderer.clear_all_touched_render_groups()

    @staticmethod
    def draw_line(renderer: RenderingManager, point: vec3, direction: vec3):
        renderer.draw_line_3d(point + direction * 200, point - direction * 200, renderer.lime())

    def main(self):
        main_path = zero
        other_path = []
        selected_point_index = 0

        while True:
            packet = GameTickPacket()
            self.game_interface.update_live_data_packet(packet)
            renderer = self.game_interface.renderer
            renderer.begin_rendering()

            points = main_path.points
            selected_point_index = min(selected_point_index, len(points) - 1)

            # select previous
            if keyboard.is_pressed("-") and selected_point_index > 0:
                selected_point_index -= 1
                time.sleep(0.1)

            # select next
            elif keyboard.is_pressed("+") and selected_point_index < len(points) - 1:
                selected_point_index += 1
                time.sleep(0.1)

            # move selected
            elif keyboard.is_pressed("left arrow"):
                points[selected_point_index][0] -= 10
                self.draw_line(renderer, points[selected_point_index], vec3(1, 0, 0))

            elif keyboard.is_pressed("right arrow"):
                points[selected_point_index][0] += 10
                self.draw_line(renderer, points[selected_point_index], vec3(1, 0, 0))

            elif keyboard.is_pressed("up arrow"):
                points[selected_point_index][1] += 10
                self.draw_line(renderer, points[selected_point_index], vec3(0, 1, 0))

            elif keyboard.is_pressed("down arrow"):
                points[selected_point_index][1] -= 10
                self.draw_line(renderer, points[selected_point_index], vec3(0, 1, 0))

            elif keyboard.is_pressed("0"):
                points[selected_point_index][2] -= 10
                self.draw_line(renderer, points[selected_point_index], vec3(0, 0, 1))

            elif keyboard.is_pressed("1"):
                points[selected_point_index][2] += 10
                self.draw_line(renderer, points[selected_point_index], vec3(0, 0, 1))

            # insert new one after selected
            elif keyboard.is_pressed("*"):
                if selected_point_index == len(points) - 1:
                    new_point = 2 * points[-1] - points[-2]
                else:
                    new_point = (points[selected_point_index] + points[selected_point_index + 1]) * 0.5

                selected_point_index += 1
                points.insert(selected_point_index, new_point)
                time.sleep(0.2)

            # delete selected
            elif keyboard.is_pressed("delete"):
                del points[selected_point_index]
                selected_point_index -= 1
                time.sleep(0.2)

            # dump points into file
            elif keyboard.is_pressed("enter"):
                with open("points.txt", "w") as file:
                    file.write("\nZERO\n")
                    for point in zero.points:
                        file.write(f'vec3({int(point[0])}, {int(point[1])}, {int(point[2])}),\n')

                print("dumped path to points.txt")
                renderer.clear_all_touched_render_groups()
                time.sleep(0.2)
                exit()

            # switch between paths
            # elif keyboard.is_pressed("9"):
            #     temp = copy(main_path)
            #     main_path = other_path[0]
            #     print(temp != main_path)
            #     other_path[0] = other_path[1]
            #     other_path[1] = temp
            #     time.sleep(0.2)
            #     continue

            # render path
            path = BezierPath(points)
            curve = Curve(path.to_points(300))
            # t = curve.find_nearest(points[min(selected_point_index, len(points) - 2)])
            # rendered_points = [curve.point_at(t + i) for i in range(-5000, 5000, 200)]
            renderer.draw_polyline_3d(curve.points, renderer.white())

            for i in range(30):
                pos = curve.point_at((1 - i / 30) * curve.length)
                renderer.draw_string_3d(pos, 1, 1, str(i), renderer.white())

            for i, point in enumerate(points):
                selected = i == selected_point_index
                color = renderer.yellow() if selected else renderer.red()
                size = 6 if selected else 4
                renderer.draw_rect_3d(point, size, size, True, color, True)

            renderer.draw_string_2d(10, 10, 1, 1, str(int(curve.length)), renderer.yellow())
            renderer.end_rendering()

            for i in range(len(other_path)):
                renderer.begin_rendering("reference" + str(i))
                blue = renderer.create_color(255, 150, 180, 255)
                # render the other path for reference
                path = BezierPath(other_path[i].points)
                curve2 = Curve(path.to_points(300))
                # rendered_points = [curve2.point_at(curve2.length - curve.length + t + i) for i in range(-5000, 5000, 200)]
                renderer.draw_polyline_3d(curve2.points, blue)

                for i in range(30):
                    pos = curve2.point_at((1 - i / 30) * curve2.length)
                    renderer.draw_string_3d(pos, 1, 1, str(i), renderer.blue())

                renderer.draw_string_2d(10, 30, 1, 1, str(int(curve2.length)), renderer.yellow())
                renderer.end_rendering()

            # render the rings of fire orbit for reference
            renderer.begin_rendering("orbit")
            points = [dot(axis_to_rotation(vec3(0, 0, i / 30 * pi * 2)), vec3(2200, 0, 1400)) for i in range(30)]
            renderer.draw_polyline_3d(points, renderer.orange())
            renderer.end_rendering()

            time.sleep(0.02)


if __name__ == "__main__":
    editor = BezierPathEditor()
    editor.main()
