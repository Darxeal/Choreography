"""
Rotates a dollycam path clockwise around the first point.

Usage:
python rotate_dolly_path.py <input file> <degrees> <output file>
"""


import json
import math
import sys


class vec3:

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __mul__(self, other):
        if isinstance(other, float) or isinstance(other, int):
            return vec3(self.x * other, self.y * other, self.z * other)
        elif isinstance(other, vec3):
            return self.x * other.x + self.y * other.y + self.z * other.z

    def __rmul__(self, other):
        return self * other

    def __truediv__(self, other):
        return vec3(self.x / other, self.y / other, self.z / other)

    def __add__(self, other):
        return vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __neg__(self):
        return vec3(-self.x, -self.y, -self.z)

    def __imul__(self, other):
        self = self * other
        return self

    def __itruediv__(self, other):
        self = self / other
        return self

    def __iadd__(self, other):
        self = self + other
        return self

    def __isub__(self, other):
        self = self - other
        return self

    def __getitem__(self, index):
        return [self.x, self.y, self.z][index]

    def __setitem__(self, index, value):
        if index == 0:
            self.x = value
        if index == 1:
            self.y = value
        if index == 2:
            self.z = value


class mat3:
    def __init__(self):
        self.data = [0] * 9

    def __getitem__(self, index):
        return self.data[index[0] + 3 * index[1]]

    def __setitem__(self, index, value):
        self.data[index[0] + 3 * index[1]] = value


def dot(a, b):
    if isinstance(b, vec3):
        Av = vec3()

        for i in range(3):
            Av[i] = 0
            for j in range(3):
                Av[i] += a[i, j] * b[j]

        return Av

    else:
        c = mat3()

        for i in range(3):
            for j in range(3):
                c[i, j] = 0.0
                for k in range(3):
                    c[i, j] += a[i, k] * b[k, j]
        return c

def norm(v: vec3) -> float:
    return math.sqrt(v.x**2 + v.y**2 + v.z**2)

def to_radians(unreal_angle: int) -> float:
    return unreal_angle * math.pi / 32768

def to_unreal_angle(radians: float) -> int:
    return float(int(radians / math.pi * 32768))

def euler_to_rotation(pyr: vec3) -> mat3:
    CP = math.cos(pyr[0])
    SP = math.sin(pyr[0])
    CY = math.cos(pyr[1])
    SY = math.sin(pyr[1])
    CR = math.cos(pyr[2])
    SR = math.sin(pyr[2])

    theta = mat3()

    # front direction
    theta[0, 0] = CP * CY
    theta[1, 0] = CP * SY
    theta[2, 0] = SP

    # left direction
    theta[0, 1] = CY * SP * SR - CR * SY
    theta[1, 1] = SY * SP * SR + CR * CY
    theta[2, 1] = -CP * SR

    # up direction
    theta[0, 2] = -CR * CY * SP - SR * SY
    theta[1, 2] = -CR * SY * SP + SR * CY
    theta[2, 2] = CP * CR

    return theta

def rotation_to_euler(theta: mat3) -> vec3:
    return vec3(
        math.atan2(theta[2, 0], norm(vec3(theta[0, 0], theta[1, 0], 0))),
        math.atan2(theta[1, 0], theta[0, 0]),
        math.atan2(-theta[2, 1], theta[2, 2])
    )

def axis_to_rotation(omega: vec3) -> mat3:
    u = omega / norm(omega)

    c = math.cos(norm(omega))
    s = math.sin(norm(omega))

    r = mat3()

    r[0, 0] = u[0]*u[0]*(1.0 - c) + c
    r[0, 1] = u[0]*u[1]*(1.0 - c) - u[2]*s
    r[0, 2] = u[0]*u[2]*(1.0 - c) + u[1]*s
    r[1, 0] = u[1]*u[0]*(1.0 - c) + u[2]*s
    r[1, 1] = u[1]*u[1]*(1.0 - c) + c
    r[1, 2] = u[1]*u[2]*(1.0 - c) - u[0]*s
    r[2, 0] = u[2]*u[0]*(1.0 - c) - u[1]*s
    r[2, 1] = u[2]*u[1]*(1.0 - c) + u[0]*s
    r[2, 2] = u[2]*u[2]*(1.0 - c) + c

    return r

def frame_rotation(frame: dict) -> mat3:
    return euler_to_rotation(vec3(
        to_radians(frame["rotation"]["pitch"]),
        to_radians(frame["rotation"]["yaw"]),
        to_radians(frame["rotation"]["roll"])
    ))


_, input_file, degrees, output_file = sys.argv

path_dict = json.load(open(input_file))
frames = list(path_dict.values())

rots = [frame_rotation(frame) for frame in frames]
points = [vec3(frame["location"]["x"], frame["location"]["y"], frame["location"]["z"]) for frame in frames]


origin = points[0]
radians = math.radians(-int(degrees))
transformation = axis_to_rotation(vec3(0, 0, radians))

new_points = [dot(transformation, p - origin) + origin for p in points]
new_rots = [dot(transformation, rot) for rot in rots]

for i in range(len(frames)):
    frame = frames[i]
    pyr = rotation_to_euler(new_rots[i])
    frame["rotation"]["pitch"] = to_unreal_angle(pyr.x)
    frame["rotation"]["yaw"] = to_unreal_angle(pyr.y)
    frame["rotation"]["roll"] = to_unreal_angle(pyr.z)
    frame["location"]["x"] = new_points[i].x
    frame["location"]["y"] = new_points[i].y
    frame["location"]["z"] = new_points[i].z

json.dump(path_dict, open(output_file, "w"), indent=4)

print("Rotated path successfully!")
