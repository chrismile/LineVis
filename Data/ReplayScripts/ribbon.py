import math
from modules.campath import camera_path_circle
import g

def init_scene():
    g.set_duration(0)
    g.set_dataset('Arched Bridge 3D (TVCG01, Fig 6, Fig 9)')
    g.set_camera_checkpoint('Closeup')
    g.set_rendering_algorithm_settings({
        'band_width': 0.005,
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

def replay():
    radius = 0.5
    total_time = 8
    center = (-0.2, -0.05, 0.0)

    init_scene()
    camera_path_circle(math.pi * 0.3, math.pi * 0.5, radius, radius, total_time=total_time, center=center)
    g.set_duration(6)
    elliptic_tubes_on()
    g.set_duration(3)
    camera_path_circle(math.pi * 0.5, math.pi * 0.7, radius, radius, total_time=total_time, center=center)
