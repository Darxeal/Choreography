from rlbot.matchconfig.match_config import MutatorConfig, MatchConfig, PlayerConfig
from rlbot.setup_manager import SetupManager
from rlbot.utils.rendering.rendering_manager import RenderingManager
from rlbot.utils.structures.game_interface import GameInterface
from rlbot.utils.structures.game_data_struct import GameTickPacket
import time
import keyboard
from choreography.choreos.dragons import PURPLE_DRAGON_PATH, BLUE_DRAGON_PATH, BezierPath
from rlutilities.simulation import Curve

from rlutilities.linear_algebra import vec3


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
        points = PURPLE_DRAGON_PATH.points
        selected_point_index = 0

        while True:
            packet = GameTickPacket()
            self.game_interface.update_live_data_packet(packet)
            renderer = self.game_interface.renderer
            renderer.begin_rendering()

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
                    for point in points:
                        file.write(f'vec3({int(point[0])}, {int(point[1])}, {int(point[2])}),\n')
                print("dumped path to points.txt")
                time.sleep(0.2)

            # render path
            path = BezierPath(points)
            curve = Curve(path.to_points(300))
            renderer.draw_polyline_3d(curve.points, renderer.white())

            for i, point in enumerate(points):
                selected = i == selected_point_index
                color = renderer.yellow() if selected else renderer.red()
                size = 6 if selected else 4
                renderer.draw_rect_3d(point, size, size, True, color, True)

            # render the other path for reference
            # renderer.begin_rendering("reference")
            renderer.draw_polyline_3d(BLUE_DRAGON_PATH.to_points(70), renderer.create_color(255, 130, 150, 255))
            renderer.end_rendering()

            time.sleep(0.02)


if __name__ == "__main__":
    editor = BezierPathEditor()
    editor.main()
