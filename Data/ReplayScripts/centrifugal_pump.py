import math
import numpy as np
import g

def init_scene():
    g.set_duration(0)
    g.set_renderer('Vulkan Ray Tracer')
    g.set_dataset('Centrifugal Pump (DES.res_t2564)')
    #g.set_camera_checkpoint('Overview Square')
    g.set_rendering_algorithm_settings({
        'line_width': 0.006,
        'band_width': 0.007,
        'depth_cue_strength': 0.4,
        'num_accumulated_frames': 1,
        'num_samples_per_frame': 16,
        'ambient_occlusion_strength': 0.75,
        'ambient_occlusion_gamma': 1.5,
        'ambient_occlusion_radius': 0.06,
        'ambient_occlusion_iterations': 1,
        'ambient_occlusion_samples_per_frame': 16,
        'ambient_occlusion_denoiser': 'OptiX Denoiser',
    })
    g.set_dataset_settings({
        'tube_num_subdivisions': 32,
        'hull_opacity': 0.0,
        'thick_bands': True,
        'rotating_helicity_bands': True,
        'use_uniform_twist_line_width': True,
        'band_subdivisions': 6,
        'separator_width': 0.2,
        'helicity_rotation_factor': 1.0,
    })
    g.set_transfer_function_range(-1000.0, 1000.0)
    g.set_duration(0.5)

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

def camera_path_circle(angle_start, angle_end, radius_start, radius_end, total_time):
    center = (0.0, 0.0, 0.0)

    # Bezier curve control points.
    p0 = (0, 0)
    p1 = (0.4, 0)
    p2 = (0.6, 1)
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

        camera_pos = (math.cos(angle) * radius + center[0], center[1], math.sin(angle) * radius + center[2])
        yaw_pitch = (math.pi + angle, 0.0)

        g.set_camera_position(camera_pos)
        g.set_camera_yaw_pitch_rad(yaw_pitch)
        g.set_duration(total_time / subdivisions)


def change_twist_line_width():
    g.set_duration(2)
    g.set_dataset_settings({
        'separator_width': 0.5,
    })
    g.set_duration(2)
    g.set_dataset_settings({
        'separator_width': 0.05,
    })
    g.set_duration(2)
    g.set_dataset_settings({
        'separator_width': 0.2,
    })
    g.set_duration(1)


def todo():
    g.set_duration(0)
    g.set_dataset_settings({
        'use_uniform_twist_line_width': True,
        'band_subdivisions': 4,
        'separator_width': 0.2,
        'helicity_rotation_factor': 1.0,
    })


def replay():
    init_scene()
    #camera_path_circle(math.pi * 0.8, math.pi * 0.5, 0.50, 0.50, total_time=3)
    #camera_path_circle(math.pi * 0.5, math.pi * 0.3, 0.50, 0.19, total_time=4)
    #camera_path_circle(math.pi * 0.3, math.pi * 0.7, 0.19, 0.19, total_time=4)
    g.set_duration(1)
    change_twist_line_width()
