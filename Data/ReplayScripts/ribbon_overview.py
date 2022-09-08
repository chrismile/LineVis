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
    start_position = (0, 0, 1.75)
    start_center = (0, 0, 0)
    start_yaw = -1.5708
    start_pitch = 0.0
    init_scene(start_position, start_center, start_yaw, start_pitch)
    g.set_duration(2)

    g.set_duration(1)
    jitter_camera(start_center, start_position, math.pi * 1.5, 0.0)
    g.set_duration(1)

    elliptic_tubes_on()

    g.set_duration(1)
    jitter_camera(start_center, start_position, math.pi * 1.5, 0.0)
    g.set_duration(1)
    g.set_duration(2)
