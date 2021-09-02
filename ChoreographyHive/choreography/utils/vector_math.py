from typing import List

from rlutilities.linear_algebra import vec3, norm, normalize


def distance(position: vec3, target: vec3) -> float:
    return norm(position - target)


def direction(source: vec3, target: vec3) -> vec3:
    return normalize(target - source)


def lerp(a, b, t):
    return a + t * (b - a)


def to_list(v: vec3) -> List[float]:
    return [v[0], v[1], v[2]]
