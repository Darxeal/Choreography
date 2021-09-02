from typing import List

import numpy
from scipy import interpolate
from scipy.interpolate import InterpolatedUnivariateSpline

from rlutilities.linear_algebra import vec3, normalize, cross, axis_to_rotation, dot
from rlutilities.simulation import Curve


def to_curve(path: List[vec3], n_points=2000) -> Curve:
    x = [p[0] for p in path]
    y = [p[1] for p in path]
    z = [p[2] for p in path]

    tck, u = interpolate.splprep([x, y, z])
    new_points = interpolate.splev(numpy.linspace(0, 1, n_points), tck)
    curve_points = [vec3(new_points[0][i], new_points[1][i], new_points[2][i]) for i in range(n_points)]
    return Curve(curve_points)


def get_orientation_up(tangent: vec3, angle: float) -> vec3:
    left = normalize(cross(tangent, vec3(0, 0, 1)))
    up = normalize(cross(left, tangent))
    rot = axis_to_rotation(tangent * angle)
    return normalize(dot(rot, up))


class SplinePath:
    def __init__(self, points: List[vec3], n_points=2000):
        self.points = points
        self.curve = to_curve(points, n_points=n_points)
        self.length = self.curve.length

    def find_nearest(self, v: vec3) -> float:
        t = self.curve.find_nearest(v)
        return self.length - t

    def point_at(self, t: float) -> vec3:
        t = self.length - t
        return self.curve.point_at(t)

    def tangent_at(self, t: float) -> vec3:
        t = self.length - t
        return normalize(self.curve.tangent_at(t))


class OrientationSplinePath:
    def __init__(self, path: SplinePath, angles: List[float]) -> None:
        self.path = path
        self.angles = angles

        ts = [path.find_nearest(p) for p in path.points]
        ups = [get_orientation_up(path.tangent_at(t), angles[i]) for i, t in enumerate(ts)]
        self.x_spline = InterpolatedUnivariateSpline(ts, [up[0] for up in ups])
        self.y_spline = InterpolatedUnivariateSpline(ts, [up[1] for up in ups])
        self.z_spline = InterpolatedUnivariateSpline(ts, [up[2] for up in ups])

    def up_at(self, t: float) -> vec3:
        return normalize(vec3(self.x_spline(t), self.y_spline(t), self.z_spline(t)))


class ParameterSplinePath:
    def __init__(self, path: SplinePath, values: List[float]) -> None:
        self.path = path
        self.values = values

        ts = [path.find_nearest(p) for p in path.points]
        self.spline = InterpolatedUnivariateSpline(ts, values)

    def value_at(self, t: float) -> float:
        return float(self.spline(t))
