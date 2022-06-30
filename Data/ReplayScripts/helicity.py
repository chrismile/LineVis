import math
from modules.campath import camera_path_circle
import g

def init_scene():
    g.set_duration(0)
    g.set_renderer('Vulkan Ray Tracer')
    g.set_dataset('Vortex Street (Line #9, Subdiv)')
    g.set_camera_checkpoint('Video')
    g.set_rendering_algorithm_settings({
        'line_width': 0.0036,
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
        'use_uniform_twist_line_width': False,
        'band_subdivisions': 4,
        'separator_width': 0.2,
        'helicity_rotation_factor': 1.0,
    })
    g.set_transfer_function_range(-20.0, 20.0)
    g.set_duration(0.5)

def change_twist_line_width():
    g.set_duration(2)
    g.set_dataset_settings({
        'separator_width': 0.4,
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

def change_twist_line_frequency():
    g.set_duration(2)
    g.set_dataset_settings({
        'helicity_rotation_factor': 2.0,
    })
    g.set_duration(3)
    g.set_dataset_settings({
        'helicity_rotation_factor': 0.5,
    })
    g.set_duration(1)
    g.set_dataset_settings({
        'helicity_rotation_factor': 1.0,
    })
    g.set_duration(1)

def enable_uniform_twist_line_width():
    g.set_duration(0)
    g.set_dataset_settings({
        'use_uniform_twist_line_width': True,
        'band_subdivisions': 4,
    })


def replay():
    init_scene()

    g.set_duration(8)
    #change_twist_line_frequency()
    g.set_duration(2)
    g.set_dataset_settings({
        'helicity_rotation_factor': 2.0,
    })
    g.set_duration(2)
    #change_twist_line_width()

    g.set_duration(2)
    g.set_dataset_settings({
        'separator_width': 0.4,
    })
    g.set_duration(2)

    g.set_duration(2)
    g.set_dataset_settings({
        'helicity_rotation_factor': 1.0,
    })
    g.set_duration(8)

    enable_uniform_twist_line_width()
    g.set_duration(3)

    change_twist_line_frequency()
    change_twist_line_width()
    g.set_duration(2)
