from choreography.choreography_group import ChoreographyGroupFactory

factory = ChoreographyGroupFactory()

phoenix = factory.make_group(count=13, team=1, appearance="phoenix-wings.cfg")
for i in [0, 3, 7]:
    phoenix.appearances[i] = "phoenix-body.cfg"

air_force_red = factory.make_group(count=3, team=1, appearance="royal_air_force_red.cfg")
air_force_white = factory.make_group(count=3, team=1, appearance="royal_air_force_white.cfg")
air_force_blue = factory.make_group(count=3, team=1, appearance="royal_air_force_blue.cfg")

air_force = air_force_red + air_force_white + air_force_blue

purple_dragon = factory.make_group(count=5, team=0, appearance="purple_dragon.cfg")
blue_dragon = factory.make_group(count=5, team=0, appearance="blue_dragon.cfg")

shape_shifter = factory.make_group(count=5, team=0, appearance="shape_shifter_orange.cfg")
shape_shifter_ext = factory.make_group(count=5, team=0, appearance="shape_shifter_blue.cfg")

# needs to be added in the order it was created! TODO: make that not necessary
all_groups = phoenix + air_force + purple_dragon + blue_dragon + shape_shifter + shape_shifter_ext
