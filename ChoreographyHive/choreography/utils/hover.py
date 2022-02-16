from rlutilities.mechanics import Reorient
from rlutilities.simulation import Car, Input

from rlutilities.linear_algebra import vec3, normalize, look_at, norm


class Hover:
    """
    PD controller for hovering in the air
    """
    P = 7.0
    D = 2.5

    def __init__(self, car: Car):
        self.reorient = Reorient(car)
        self.target: vec3 = None
        self.car: Car = car
        self.up: vec3 = None
        self.controls: Input = Input()

    def step(self, dt):
        delta_target = self.target - self.car.position
        clamped_delta = delta_target
        if norm(delta_target) > 300:
            clamped_delta = normalize(delta_target) * 300

        target_direction = normalize(vec3(
            (clamped_delta[0]) * self.P - self.car.velocity[0] * self.D,
            (clamped_delta[1]) * self.P - self.car.velocity[1] * self.D,
            1000
        ))

        self.reorient.target_orientation = look_at(target_direction, self.up)

        self.reorient.step(dt)
        self.controls = self.reorient.controls
        self.controls.boost = 0

        # tap boost to keep height
        if (delta_target[2] - self.car.velocity[2] * 0.5) > 0:
            self.controls.boost = 1

        self.controls.jump = False
        if self.car.position.z < 100:
            self.controls.jump = True
        if 150 < self.car.position.z < 200:
            self.controls.jump = True
            self.controls.pitch = 0
            self.controls.yaw = 0
            self.controls.roll = 0
