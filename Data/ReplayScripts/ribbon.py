import math
import numpy as np
import g

def init_scene():
    g.set_duration(0)
    g.set_dataset('Arched Bridge 3D (TVCG01, Fig 6, Fig 9)')
    g.set_camera_checkpoint('Closeup')
    g.set_rendering_algorithm_settings({
        'band_width': 0.003,
        'depth_cue_strength': 0.8
    })
    g.set_dataset_settings({
        'major_on': False,
        'medium_on': False,
        'minor_on': True,
        'minor_lod': 0.3,
        'minor_use_bands': True,
        'thick_bands': False,
        'smoothed_bands': True,
        'use_principal_stress_direction_index': True
    })
    g.set_transfer_functions(['qualitative-ocher.xml', 'qualitative-emerald.xml', 'qualitative-pale-lilac.xml'])

def elliptic_tubes_on():
    g.set_duration(0)
    g.set_dataset_settings({
        'thick_bands': True
    })

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

def camera_path_circle(start_angle, end_angle):
    center = (-0.2, -0.05, 0.0)
    radius = 0.5

    # Bezier curve control points.
    p0 = (0, 0)
    p1 = (0.4, 0)
    p2 = (0.6, 1)
    p3 = (1, 1)

    total_time = 8
    subdivisions = 256

    def f(t):
        return t * (3 + t * (-6 + t * 4))

    g.set_duration(0.0)
    for i in range(subdivisions + 1):
        t = f_cubic_bezier(i / subdivisions, p0, p1, p2, p3)
        time = t * total_time
        angle = start_angle + t * (end_angle - start_angle)

        camera_pos = (math.cos(angle) * radius + center[0], center[1], math.sin(angle) * radius + center[2])
        yaw_pitch = (math.pi + angle, 0.0)

        g.set_camera_position(camera_pos)
        g.set_camera_yaw_pitch_rad(yaw_pitch)
        g.set_duration(total_time / subdivisions)


def replay():
    init_scene()
    camera_path_circle(math.pi * 0.3, math.pi * 0.5)
    g.set_duration(6)
    elliptic_tubes_on()
    g.set_duration(3)
    camera_path_circle(math.pi * 0.5, math.pi * 0.7)
