import math
from functools import partial
from modules.campath import camera_path_circle, jitter_camera, camera_pitch_rotation_smooth
import g

def test_eaw(color_phi, pos_phi, normal_phi, iters):
    g.set_duration(1)
    g.set_duration(0)
    g.set_rendering_algorithm_settings({
        'ambient_occlusion_denoiser': 'EAW',
        'eaw_color_phi':  color_phi,
        'eaw_pos_phi':    pos_phi,
        'eaw_normal_phi': normal_phi,
        'eaw_num_iters':  iters,
    })
    g.save_screenshot("EAW-cp{:0.3f}-pp{:0.3f}-np{:0.3f}-iters{}".format(color_phi, pos_phi, normal_phi, iters))
    g.set_duration(0)

def replay():
    start_look_at = (0.0, 0.0, 0.0)
    start_position = (0.0478069, 0.0157976, -0.140504)
    start_yaw = 1.90416
    start_pitch = -0.286795
    FoVy = 53.1301

    g.set_duration(0)
    g.set_renderer('Vulkan Ray Tracer')
    g.set_dataset('Mesh: Barber Shop')
    g.set_camera_fovy_deg(FoVy)
    g.set_camera_position(start_position)
    g.set_camera_yaw_pitch_rad(start_yaw, start_pitch)
    g.set_camera_look_at_location(start_look_at)
    g.set_transfer_function_range(-100000, -100000)
    g.set_transfer_function('white.xml')

    g.set_rendering_algorithm_settings({
        'depth_cue_strength': 0.001,
        'num_accumulated_frames': 1,
        'num_samples_per_frame': 10,
        'ambient_occlusion_strength': 1,
        'ambient_occlusion_gamma': 1.5,
        'ambient_occlusion_radius': 0.06,
        'ambient_occlusion_iterations': 1,
        'ambient_occlusion_samples_per_frame': 10,
    })
    g.set_duration(0)

    # test pp between 0.01 and 0.2
    # for pp in range(10, 15, 5):
    for cp in range(0, 1005, 5):
        for iter in range(1, 6):
            test_eaw(cp/1000, 0.16, 0.06, iter)
