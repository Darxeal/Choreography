import math
from typing import List

from rlutilities.linear_algebra import vec3, norm, normalize, clip, dot


def distance(position: vec3, target: vec3) -> float:
    return norm(position - target)


def direction(source: vec3, target: vec3) -> vec3:
    return normalize(target - source)


# stolen from RLU (It wasn't exposed to python and I'm too lazy to recompile it)
class CubicBezierCurve:

    def __init__(self, p0: vec3, p1: vec3, p2: vec3, p3: vec3):
        self.p0 = p0
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3

    def position(self, t: float) -> vec3:
        t = clip(t, 0, 1)
        s = 1 - t
        return s * s * s * self.p0 + 3 * s * s * t * self.p1 + 3 * s * t * t * self.p2 + t * t * t * self.p3

    def tangent(self, t: float) -> vec3:
        t = clip(t, 0, 1)
        s = 1.0 - t
        return 3 * s * s * (self.p1 - self.p0) + 6 * s * t * (self.p2 - self.p1) + 3 * t * t * (self.p3 - self.p2)

    def t_nearest(self, pos: vec3) -> float:
        t = 0.5
        for i in range(6):
            r = self.position(t) - pos
            dr = self.tangent(t)
            if abs(dot(r, normalize(dr))) < 10.0:
                break
            t -= dot(r, dr) / dot(dr, dr)
        return t

    def to_points(self, n: int) -> List[vec3]:
        return [self.position(i / n) for i in range(n)]


class BezierPath:

    def __init__(self, points: List[vec3]):
        assert len(points) >= 4
        self.curves: List[CubicBezierCurve] = []
        self.points = points

        for i in range(1, len(points) - 2):
            p0 = points[i]
            p3 = points[i + 1]
            p1 = p0 + direction(points[i - 1], p3) * distance(p0, p3) / 3
            p2 = p3 - direction(p0, points[i + 2]) * distance(p0, p3) / 3
            self.curves.append(CubicBezierCurve(p0, p1, p2, p3))

    def position(self, t: float) -> vec3:
        n = len(self.curves)
        i = clip(math.floor(t * n), 0, n - 1)
        return self.curves[i].position((t - i / n) * n)

    def to_points(self, n: int) -> List[vec3]:
        return [self.position(i / n) for i in range(n)]
