from rlutilities.linear_algebra import vec3, norm, normalize


def distance(position: vec3, target: vec3) -> float:
    return norm(position - target)


def direction(source: vec3, target: vec3) -> vec3:
    return normalize(target - source)
