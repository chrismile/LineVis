import math
from modules.campath import camera_path_circle, jitter_camera, blend_camera_view
import g

def init_scene(start_position, start_center, start_yaw, start_pitch):
    g.set_duration(0)
    g.set_renderer('Opaque')
    g.set_dataset('Arched Bridge 3D (TVCG01, Fig 6, Fig 9)')
    g.set_camera_fovy_deg(10.0)
    g.set_camera_look_at_location(start_center)
    g.set_camera_position(start_position)
    g.set_camera_yaw_pitch_rad(start_yaw, start_pitch)
    #g.set_camera_checkpoint('Closeup')
    g.set_rendering_algorithm_settings({
        'band_width': 0.005,
        'depth_cue_strength': 0.8,
        'num_samples': 2
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
    g.set_rendering_algorithm_settings({
        'num_samples': 4
    })

def replay():
    radius = 0.5
    total_time = 8
    center = (-0.2, -0.05, 0.0)

    start_position = (0, 0, 1.86673)
    start_center = (0, 0, 0)
    start_yaw = -1.5708
    start_pitch = 0.0
    init_scene(start_position, start_center, start_yaw, start_pitch)
    g.set_duration(4)

    g.set_duration(1)
    jitter_camera(start_center, start_position, math.pi * 1.5, 0.0)
    g.set_duration(1)

    g.set_duration(2)
    closeup_position = (0.0938926, -0.05, 0.404509)
    closeup_yaw = -2.19911
    closeup_pitch = 0.0
    diff_vec = (
        start_center[0] - start_position[0],
        start_center[1] - start_position[1],
        start_center[2] - start_position[2])
    start_radius = math.sqrt(diff_vec[0] * diff_vec[0] + diff_vec[1] * diff_vec[1] + diff_vec[2] * diff_vec[2])
    camera_path_circle(
        angle_start=start_yaw - math.pi, angle_end=closeup_yaw - math.pi,
        radius_start=start_radius, radius_end=start_radius,
        total_time=4, center=start_center, acceleration=0.2)
    g.set_duration(2)
    g.set_duration(4)
    g.set_camera_position(closeup_position)
    g.set_duration(2)

    #g.set_duration(2)
    #g.set_duration(6)
    #g.set_camera_position()
    #g.set_camera_yaw_pitch_rad(-2.19911, 0.0)

    g.set_duration(0)
    g.set_camera_look_at_location((-0.2, -0.05, 0.0))
    g.set_duration(0)

    camera_path_circle(math.pi * 0.3, math.pi * 0.5, radius, radius, total_time=total_time, center=center)

    #g.set_duration(1)
    #jitter_camera(center, None, math.pi * 1.5, 0.0, radius=radius)
    #g.set_duration(1)

    g.set_duration(6)
    elliptic_tubes_on()

    #g.set_duration(1)
    #jitter_camera(center, None, math.pi * 1.5, 0.0, radius=radius)
    #g.set_duration(1)

    g.set_duration(3)
    camera_path_circle(math.pi * 0.5, math.pi * 0.7, radius, radius, total_time=total_time, center=center)

    g.set_duration(1)
    camera_path_circle(math.pi * 0.7, math.pi * 0.5, radius, radius, total_time=2, center=center)
    g.set_duration(2)
    g.set_camera_position(start_position)
    g.set_duration(1)

    g.set_duration(0)
    g.set_camera_look_at_location(start_center)
    g.set_duration(0)

    g.set_duration(1)
    jitter_camera(start_center, start_position, math.pi * 1.5, 0.0)
    g.set_duration(1)
    g.set_duration(6)
