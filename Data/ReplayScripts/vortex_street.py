import math
from modules.campath import camera_path_circle
import g

def init_scene(start_look_at, start_position, start_yaw, start_pitch):
    g.set_duration(0)
    g.set_renderer('Vulkan Ray Tracer')
    g.set_dataset('Vortex Street')
    g.set_camera_fovy_deg(40.0)
    g.set_camera_position(start_position)
    g.set_camera_yaw_pitch_rad(start_yaw, start_pitch)
    g.set_camera_look_at_location(start_look_at)
    #g.set_camera_checkpoint('Overview Square')
    g.set_rendering_algorithm_settings({
        'line_width': 0.004,
        'band_width': 0.005,
        'depth_cue_strength': 0.4,
        'num_accumulated_frames': 1,
        'num_samples_per_frame': 32,
        'use_deterministic_sampling': False,
        'ambient_occlusion_strength': 0.75,
        'ambient_occlusion_gamma': 1.5,
        'ambient_occlusion_radius': 0.1,
        'ambient_occlusion_iterations': 1,
        'ambient_occlusion_samples_per_frame': 32,
        'ambient_occlusion_denoiser': 'OptiX Denoiser',
    })
    g.set_dataset_settings({
        'tube_num_subdivisions': 32,
        'hull_opacity': 0.2,
        'thick_bands': True,
        'min_band_thickness': 0.15,
        'use_ribbons': True,
        'rotating_helicity_bands': False,
        'use_uniform_twist_line_width': True,
        'band_subdivisions': 6,
        'separator_width': 0.2,
        'helicity_rotation_factor': 1.2,
    })
    g.set_transfer_function_range(-15.0, 15.0)
    g.set_duration(0.5)


def change_twist_line_width():
    g.set_duration(4)
    g.set_dataset_settings({
        'separator_width': 0.8,
    })
    g.set_duration(2)
    g.set_dataset_settings({
        'separator_width': 0.05,
    })
    g.set_duration(2)
    g.set_dataset_settings({
        'separator_width': 0.3,
    })
    g.set_duration(1)


def change_twist_line_frequency():
    g.set_duration(2)
    g.set_dataset_settings({
        'helicity_rotation_factor': 2.0,
    })
    g.set_duration(8)
    g.set_dataset_settings({
        'helicity_rotation_factor': 0.5,
    })
    g.set_duration(8)
    g.set_dataset_settings({
        'helicity_rotation_factor': 1.0,
    })
    g.set_duration(1)


def camera_path_overview():
    radius_outer = 0.55
    scene_center = (0.0, -0.02, 0.0)
    camera_pitch = math.pi * 0.08
    camera_path_circle(
        angle_start=math.pi * 0.6, angle_end=math.pi * 2.6, radius_start=radius_outer, radius_end=radius_outer,
        total_time=12, pitch=camera_pitch, center=scene_center, acceleration=0.2)

def jitter_camera(center, position, yaw, pitch):
    pitch *= -1
    angle_std = yaw - math.pi
    angle_start = angle_std - 0.1
    angle_end = angle_std + 0.1
    diff_vec = (center[0] - position[0], center[1] - position[1], center[2] - position[2])
    radius = math.sqrt(diff_vec[0] * diff_vec[0] + diff_vec[1] * diff_vec[1] + diff_vec[2] * diff_vec[2])
    camera_path_circle(
        angle_start=angle_std, angle_end=angle_end, radius_start=radius, radius_end=radius,
        total_time=1, pitch=pitch, center=center, acceleration=0.2)
    for i in range(1):
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


def show_aliasing(closeup_center, closeup_position, closeup_yaw, closeup_pitch):
    g.set_duration(0)
    g.set_rendering_algorithm_settings({
        'num_samples_per_frame': 2,
        'use_deterministic_sampling': True,
    })
    g.set_dataset_settings({
        'min_band_thickness': 0.01,
    })
    g.set_duration(0)
    g.set_duration(1)
    jitter_camera(closeup_center, closeup_position, closeup_yaw, closeup_pitch)
    g.set_duration(1)

    g.set_duration(0)
    g.set_rendering_algorithm_settings({
        'num_samples_per_frame': 32,
        'use_deterministic_sampling': False,
    })
    g.set_dataset_settings({
        'min_band_thickness': 0.15,
    })
    g.set_duration(0)
    g.set_duration(1)
    jitter_camera(closeup_center, closeup_position, closeup_yaw, closeup_pitch)
    g.set_duration(1)


def replay():
    start_look_at = (0.0, 0.0, 0.0)
    start_position = (-0.16462, 0.116779, 0.506648)
    start_yaw = -1.25664
    start_pitch = -0.251327

    init_scene(start_look_at, start_position, start_yaw, start_pitch)

    camera_path_overview()

    closeup_center = (-0.0111691, -0.00516945, -0.0130456)
    closeup_position = (-0.19656, -0.00477941, 0.15662)
    closeup_yaw = -0.741139
    closeup_pitch = -0.00155201
    g.set_duration(2)
    g.set_camera_position(closeup_position)
    g.set_camera_yaw_pitch_rad(closeup_yaw, closeup_pitch)
    g.set_duration(1)
    g.set_camera_look_at_location(closeup_center)
    g.set_duration(0)

    show_aliasing(closeup_center, closeup_position, closeup_yaw, closeup_pitch)

    g.set_duration(0)
    g.set_dataset_settings({
        'rotating_helicity_bands': True,
    })
    g.set_duration(0)
    g.set_duration(2)

    g.set_duration(2)
    g.set_camera_position(start_position)
    g.set_camera_yaw_pitch_rad(start_yaw, start_pitch)
    g.set_camera_look_at_location(start_look_at)
    g.set_duration(2)

    change_twist_line_width()
    g.set_duration(2)

    camera_path_overview()
    g.set_duration(2)
