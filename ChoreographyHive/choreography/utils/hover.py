from rlutilities.mechanics import AerialTurn
from rlutilities.simulation import Car, Input

from rlutilities.linear_algebra import vec3, normalize, look_at


class Hover:
    """
    PD controller for hovering in the air
    """
    P = 7.0
    D = 2.5

    def __init__(self, car: Car):
        self.turn = AerialTurn(car)
        self.target: vec3 = None
        self.car: Car = car
        self.up: vec3 = None
        self.controls: Input = Input()

    def step(self, dt):
        delta_target = self.target - self.car.position
        target_direction = normalize(vec3(
            (delta_target[0]) * self.P - self.car.velocity[0] * self.D,
            (delta_target[1]) * self.P - self.car.velocity[1] * self.D,
            1000
        ))

        self.turn = AerialTurn(self.car)
        self.turn.target = look_at(target_direction, self.up)

        self.turn.step(dt)
        self.controls = self.turn.controls
        self.controls.boost = 0

        # tap boost to keep height
        if (delta_target[2] - self.car.velocity[2] * 0.5) > 0:
            self.controls.boost = 1
