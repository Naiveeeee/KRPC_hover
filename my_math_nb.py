import math
import numba as nb


@nb.jit()
def cross_product(u, v):
    return (u[1]*v[2] - u[2]*v[1],
            u[2]*v[0] - u[0]*v[2],
            u[0]*v[1] - u[1]*v[0])


@nb.jit()
def dot_product(u, v):
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]


@nb.jit()
def magnitude(v):
    return math.sqrt(dot_product(v, v))


@nb.jit()
def angle_between_vectors(u, v):
    """ Compute the angle between vector u and v """
    dp = dot_product(u, v)
    if dp == 0:
        return 0
    um = magnitude(u)
    vm = magnitude(v)
    return math.acos(dp / (um*vm))


@nb.jit()
def zy_2_yp(roll,ez,ey):
    e_yaw = ez * math.cos(roll) - ey * math.sin(roll)
    e_pitch = ez * math.sin(roll) + ey * math.cos(roll)
    return (e_pitch,e_yaw)
