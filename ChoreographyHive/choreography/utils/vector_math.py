from typing import List

from rlutilities.linear_algebra import vec3, norm, normalize, clip


def distance(position: vec3, target: vec3) -> float:
    return norm(position - target)


def direction(source: vec3, target: vec3) -> vec3:
    return normalize(target - source)


def lerp(a, b, t):
    return a + t * (b - a)


def smoothstep(a, b, t):
    # Scale, bias and saturate x to 0..1 range
    x = clip((t - a) / (b - a), 0.0, 1.0)
    # Evaluate polynomial
    return x * x * x * (x * (x * 6 - 15) + 10)


def smoothlerp(a, b, t):
    return lerp(a, b, smoothstep(0, 1, t))


def to_list(v: vec3) -> List[float]:
    return [v[0], v[1], v[2]]
