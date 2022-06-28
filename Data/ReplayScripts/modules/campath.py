import math
import numpy as np
import g

def f_cubic_bezier(x, p0, p1, p2, p3):
    """
    Returns the y value of a cubic bezier curve for the given x coordinate.
    This function assumes that NumPy is installed (which is used for solving a cubic equation).
    :param x: The x coordinate of the bezier curve.
    :param p0: Control point 0.
    :param p1: Control point 1.
    :param p2: Control point 2.
    :param p3: Control point 3.
    :return: The y coordinate of the bezier curve at x.
    """
    def pow2(x):
        return x * x

    def pow3(x):
        return x * x * x

    b0 = p0[0] - x
    b1 = -3 * p0[0] + 3 * p1[0]
    b2 = 3 * p0[0] - 6 * p1[0] + 3 * p2[0]
    b3 = -p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]
    poly = np.polynomial.Polynomial((b0, b1, b2, b3), domain=[0,1], window=[0,1])
    roots = poly.roots()
    num_real = np.count_nonzero(np.isreal(roots))

    if num_real == 1:
        indices = np.nonzero(np.isreal(roots))
        t = np.real(roots[indices])
    elif num_real == 3:
        t = roots[1]
    else:
        raise Exception('Error: Invalid number of non-zero values.')

    y = pow3(1 - t) * p0[1] + 3 * pow2(1 - t) * t * p1[1] + 3 * (1 - t) * pow2(t) * p2[1] + pow3(t) * p3[1]
    return y


def camera_path_circle(
        angle_start, angle_end, radius_start, radius_end, total_time,
        pitch=0.0, center=(0.0, 0.0, 0.0), acceleration=0.4):
    # Bezier curve control points.
    p0 = (0, 0)
    p1 = (0.0 + acceleration, 0)
    p2 = (1.0 - acceleration, 1)
    p3 = (1, 1)

    subdivisions = 256

    def f(t):
        return t * (3 + t * (-6 + t * 4))

    g.set_duration(0.0)
    for i in range(subdivisions + 1):
        t = f_cubic_bezier(i / subdivisions, p0, p1, p2, p3)
        time = t * total_time
        angle = angle_start + t * (angle_end - angle_start)
        radius = radius_start + t * (radius_end - radius_start)

        if pitch == 0.0:
            camera_pos = (math.cos(angle) * radius + center[0], center[1], math.sin(angle) * radius + center[2])
            yaw_pitch = (math.pi + angle, 0.0)
        else:
            camera_pos = (
                math.cos(angle) * radius * math.cos(pitch) + center[0],
                math.sin(pitch) * radius + center[1],
                math.sin(angle) * radius * math.cos(pitch) + center[2])
            yaw_pitch = (math.pi + angle, -pitch)

        g.set_camera_position(camera_pos)
        g.set_camera_yaw_pitch_rad(yaw_pitch)
        g.set_duration(total_time / subdivisions)
