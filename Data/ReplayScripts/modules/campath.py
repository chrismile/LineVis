import math
import numpy as np
import g


def pow2(x):
    return x * x


def pow3(x):
    return x * x * x


def f_cubic_bezier_numpy(x, p0, p1, p2, p3):
    """
    Returns the y value of a cubic Bezier curve for the given x coordinate.
    This function assumes that NumPy is installed (which is used for solving a cubic equation).
    :param x: The x coordinate of the Bezier curve.
    :param p0: Control point 0.
    :param p1: Control point 1.
    :param p2: Control point 2.
    :param p3: Control point 3.
    :return: The y coordinate of the Bezier curve at x.
    """
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


def find_root(x, b0, b1, b2, b3):
    """
    Finds a root of the cubic polynomial with the passed coefficients.
    :param x: The x coordinate of the Bezier curve.
    :param b0: Cubic polynomial coefficient 0.
    :param b1: Cubic polynomial coefficient 1.
    :param b2: Cubic polynomial coefficient 2.
    :param b3: Cubic polynomial coefficient 3.
    :return: The t parameter of the Bezier curve at x.
    """
    # Use Newton's method to find the root of the polynomial: https://en.wikipedia.org/wiki/Newton%27s_method
    epsilon = 1e-6
    epsilon2 = 1e-7
    t = x
    for i in range(8):
        x_new = b0 + b1 * t + b2 * pow2(t) + b3 * pow3(t)
        if abs(x_new) < epsilon:
            return t
        xprime_new = b1 + 2 * b2 * t + 3.0 * b3 * pow2(t)
        if abs(xprime_new) < epsilon2:
            break
        t = t - x_new / xprime_new

    # If we found no root after 8 iterations, use the Bisection method: https://en.wikipedia.org/wiki/Bisection_method
    t0 = 0.0
    t1 = 1.0
    t = x
    while t0 < t1:
        x_new = b0 + b1 * t + b2 * pow2(t) + b3 * pow3(t)
        if abs(x_new) < epsilon:
            return t
        if x_new < 0.0:
            t0 = t
        else:
            t1 = t
        t = 0.5 * (t1 - t0) + t0

    # Could not find any root of the polynomial.
    return None

def f_cubic_bezier(x, p0, p1, p2, p3):
    """
    Returns the y value of a cubic Bezier curve for the given x coordinate.
    :param x: The x coordinate of the Bezier curve.
    :param p0: Control point 0.
    :param p1: Control point 1.
    :param p2: Control point 2.
    :param p3: Control point 3.
    :return: The y coordinate of the Bezier curve at x.
    """
    b0 = p0[0] - x
    b1 = -3 * p0[0] + 3 * p1[0]
    b2 = 3 * p0[0] - 6 * p1[0] + 3 * p2[0]
    b3 = -p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]
    t = find_root(x, b0, b1, b2, b3)

    if t is None:
        raise Exception('Error: Could not find a root of the polynomial.')

    y = pow3(1 - t) * p0[1] + 3 * pow2(1 - t) * t * p1[1] + 3 * (1 - t) * pow2(t) * p2[1] + pow3(t) * p3[1]
    return y


def camera_path_circle(
        angle_start, angle_end, radius_start, radius_end, total_time,
        pitch=0.0, center=(0.0, 0.0, 0.0), acceleration=0.4, acceleration_start=None, acceleration_end=None,
        radius_functor=None):
    # Bezier curve control points.
    if acceleration_start is None:
        acceleration_start = acceleration
    if acceleration_end is None:
        acceleration_end = acceleration
    p0 = (0, 0)
    p1 = (0.0 + acceleration_start, 0)
    p2 = (1.0 - acceleration_end, 1)
    p3 = (1, 1)

    subdivisions = 256

    def f(t):
        return t * (3 + t * (-6 + t * 4))

    g.set_duration(0.0)
    for i in range(subdivisions + 1):
        t = f_cubic_bezier(i / subdivisions, p0, p1, p2, p3)
        time = t * total_time
        angle = angle_start + t * (angle_end - angle_start)
        if radius_functor is None:
            radius = radius_start + t * (radius_end - radius_start)
        else:
            radius = radius_functor(t)

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


def camera_pitch_rotation_smooth(yaw, pitch_start, pitch_end, total_time, acceleration=0.4):
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
        pitch = pitch_start + t * (pitch_end - pitch_start)
        yaw_pitch = (yaw, pitch)
        g.set_camera_yaw_pitch_rad(yaw_pitch)
        g.set_duration(total_time / subdivisions)


def jitter_camera(center, position, yaw, pitch, num_iterations=1, radius=None):
    pitch *= -1
    angle_std = yaw - math.pi
    angle_start = angle_std - 0.1
    angle_end = angle_std + 0.1
    if position is not None:
        diff_vec = (center[0] - position[0], center[1] - position[1], center[2] - position[2])
        radius = math.sqrt(diff_vec[0] * diff_vec[0] + diff_vec[1] * diff_vec[1] + diff_vec[2] * diff_vec[2])
    camera_path_circle(
        angle_start=angle_std, angle_end=angle_end, radius_start=radius, radius_end=radius,
        total_time=1, pitch=pitch, center=center, acceleration=0.2)
    for i in range(num_iterations):
        camera_path_circle(
            angle_start=angle_end, angle_end=angle_start, radius_start=radius, radius_end=radius,
            total_time=2, pitch=pitch, center=center, acceleration=0.2)
        camera_path_circle(
            angle_start=angle_start, angle_end=angle_end, radius_start=radius, radius_end=radius,
            total_time=2, pitch=pitch, center=center, acceleration=0.2)
    camera_path_circle(
        angle_start=angle_end, angle_end=angle_start, radius_start=radius, radius_end=radius,
        total_time=2, pitch=pitch, center=center, acceleration=0.2)
    camera_path_circle(
        angle_start=angle_start, angle_end=angle_std, radius_start=radius, radius_end=radius,
        total_time=1, pitch=pitch, center=center, acceleration=0.2)


def blend_camera_view(
        start_pos, end_pos, start_yaw, end_yaw, start_pitch=0.0, end_pitch=0.0,
        total_time=2.0, acceleration=0.4, p1_pos=None, p2_pos=None,
        p1_yaw=None, p2_yaw=None, p1_pitch=None, p2_pitch=None):
    # Bezier curve control points.
    p0_pos = (0, 0)
    if p1_pos is None:
        p1_pos = (0.0 + acceleration, 0)
    if p2_pos is None:
        p2_pos = (1.0 - acceleration, 1)
    p3_pos = (1, 1)

    p0_yaw = (0, 0)
    if p1_yaw is None:
        p1_yaw = (0.0 + acceleration, 0)
    if p2_yaw is None:
        p2_yaw = (1.0 - acceleration, 1)
    p3_yaw = (1, 1)

    p0_pitch = (0, 0)
    if p1_pitch is None:
        p1_pitch = (0.0 + acceleration, 0)
    if p2_pitch is None:
        p2_pitch = (1.0 - acceleration, 1)
    p3_pitch = (1, 1)

    subdivisions = 256

    def f(t):
        return t * (3 + t * (-6 + t * 4))

    g.set_duration(0.0)
    for i in range(subdivisions + 1):
        t_pos = f_cubic_bezier(i / subdivisions, p0_pos, p1_pos, p2_pos, p3_pos)
        t_yaw = f_cubic_bezier(i / subdivisions, p0_yaw, p1_yaw, p2_yaw, p3_yaw)
        t_pitch = f_cubic_bezier(i / subdivisions, p0_pitch, p1_pitch, p2_pitch, p3_pitch)
        camera_pos = (
            start_pos[0] + t_pos * (end_pos[0] - start_pos[0]),
            start_pos[1] + t_pos * (end_pos[1] - start_pos[1]),
            start_pos[2] + t_pos * (end_pos[2] - start_pos[2]),
        )
        yaw = start_yaw + t_yaw * (end_yaw - start_yaw)
        pitch = start_pitch + t_pitch * (end_pitch - start_pitch)

        #yaw_pitch = (math.pi + yaw, -pitch)

        g.set_camera_position(camera_pos)
        g.set_camera_yaw_pitch_rad(yaw, pitch)
        #g.set_camera_yaw_pitch_rad(yaw_pitch)
        g.set_duration(total_time / subdivisions)
