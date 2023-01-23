import math
from functools import partial
from modules.campath import camera_path_circle, jitter_camera, camera_pitch_rotation_smooth
import g

def init_scene(start_look_at, start_position, start_yaw, start_pitch):
    g.set_duration(0)
    g.set_renderer('Deferred Opaque')
    g.set_dataset('Centrifugal Pump (DES.res_t2564)')
    g.set_camera_fovy_deg(math.atan(1.0/2.0) * 2.0 / math.pi * 180.0)
    g.set_camera_position(start_position)
    g.set_camera_yaw_pitch_rad(start_yaw, start_pitch)
    g.set_camera_look_at_location(start_look_at)
    #g.set_camera_checkpoint('Overview Square')
    g.set_rendering_algorithm_settings({
        'line_width': 0.006,
        'band_width': 0.007,
        'depth_cue_strength': 0.4,
        'supersampling': 2
    })
    g.set_dataset_settings({
        'use_capped_tubes' : True,
        'tube_num_subdivisions': 32,
        'hull_opacity': 0.0,
        'thick_bands': True,
        'rotating_helicity_bands': True,
        'use_uniform_twist_line_width': True,
        'band_subdivisions': 6,
        'separator_width': 0.2,
        'helicity_rotation_factor': 1.0,
        'use_multi_var_rendering': False,
        'selected_multi_vars_string': ''
    })
    g.set_transfer_function_range(-1000.0, 1000.0)
    g.set_duration(0.5)


def change_twist_line_frequency():
    g.set_duration(2)
    g.set_duration(0)
    g.set_dataset_settings({
        'helicity_rotation_factor': 2.0,
    })
    g.set_duration(2)
    g.set_duration(0)
    g.set_dataset_settings({
        'helicity_rotation_factor': 0.5,
    })
    g.set_duration(2)
    g.set_duration(0)
    g.set_dataset_settings({
        'helicity_rotation_factor': 1.0,
    })
    g.set_duration(4)


def multivar():
    g.set_duration(0)
    g.set_dataset_settings({
        'use_multi_var_rendering': True,
        'selected_multi_vars_string': 'Turbulence_Kinetic5'
    })
    # Helicity, Pressure, Total_Pressure, Total_Pressure_in_4, Turbulence_Kinetic5, Velocity Magnitude, Vorticity Magnitude
    g.set_transfer_functions([
        'Standard.xml', 'Standard.xml', 'greens.xml', 'greens.xml', 'reds.xml', 'Standard.xml', 'blues.xml'
    ])
    g.set_transfer_functions_ranges([
        (0.0, 1.0), (0.0, 1.0), (-32000.0, -49000.0), (-40000.0, 30000.0), (0.0, 0.6), (0.0, 1.0), (0.0, 600.0)
    ])


def radius_functor(closeup_radius, t):
    t2 = math.fmod(2.0 * t, 1.0)
    if t < 0.5:
        radius_factor = 0.7
    else:
        radius_factor = 1.0
    return closeup_radius * (1.0 + radius_factor * (1.0 - abs(1.0 - 2.0 * t2)))

def replay():
    start_look_at = (0.0, 0.0, 0.0)
    start_position = (0, 0, 0.518272)
    start_yaw = -1.5708
    start_pitch = 0.0

    init_scene(start_look_at, start_position, start_yaw, start_pitch)
    g.set_duration(5)

    closeup_center = (-0.0599098, 0.030252, 0.00189503)
    closeup_position = (-0.1093, 0.0371734, 0.21648)
    closeup_yaw = -1.34457
    closeup_pitch = -0.0314224

    g.set_duration(2)
    g.set_camera_position(closeup_position)
    g.set_camera_yaw_pitch_rad(closeup_yaw, closeup_pitch)
    g.set_duration(2)
    g.set_duration(0)
    g.set_camera_look_at_location(closeup_center)

    g.set_duration(2)
    multivar()
    g.set_duration(12)

    g.set_duration(0)
    g.set_dataset_settings({
        'selected_multi_vars_string': 'Turbulence_Kinetic5,Vorticity Magnitude'
    })
    g.set_duration(4)

    camera_pitch_rotation_smooth(closeup_yaw, closeup_pitch, closeup_pitch + math.pi / 12, 4)
    camera_pitch_rotation_smooth(closeup_yaw, closeup_pitch + math.pi / 12, closeup_pitch - math.pi / 10, 8)
    camera_pitch_rotation_smooth(closeup_yaw, closeup_pitch - math.pi / 10, closeup_pitch, 4)
    g.set_duration(4)

    change_twist_line_frequency()
    g.set_duration(2)

    g.set_duration(0)
    g.set_dataset_settings({
        'selected_multi_vars_string': 'Total_Pressure,Turbulence_Kinetic5,Vorticity Magnitude'
    })
    g.set_duration(6)

    diff_vec = (
        closeup_center[0] - closeup_position[0],
        closeup_center[1] - closeup_position[1],
        closeup_center[2] - closeup_position[2])
    closeup_radius = math.sqrt(diff_vec[0] * diff_vec[0] + diff_vec[1] * diff_vec[1] + diff_vec[2] * diff_vec[2])
    camera_path_circle(
        angle_start=closeup_yaw - math.pi, angle_end=closeup_yaw + math.pi, pitch=-closeup_pitch,
        radius_start=closeup_radius, radius_end=closeup_radius, radius_functor=partial(radius_functor, closeup_radius),
        total_time=40, center=closeup_center, acceleration=0.2)
    g.set_duration(2)

    g.set_duration(0)
    g.set_dataset_settings({
        'use_capped_tubes' : False,
        'use_ribbons': True,
        'rotating_helicity_bands': False,
    })
    g.set_duration(14)

    g.set_duration(0)
    g.set_dataset_settings({
        'use_capped_tubes' : True,
    })
    g.set_duration(0)
