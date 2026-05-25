import math
from modules.campath import camera_path_circle
import g

def init_scene(start_look_at, start_position, start_yaw, start_pitch):
    g.set_duration(0)
    #g.set_renderer('Deferred Opaque')
    g.set_dataset('Aneurysm')
    g.set_camera_fovy_deg(math.atan(1.0/2.0) * 2.0 / math.pi * 180.0)
    g.set_camera_position(start_position)
    g.set_camera_yaw_pitch_rad(start_yaw, start_pitch)
    g.set_camera_look_at_location(start_look_at)
    g.set_rendering_algorithm_settings({
        'reset_accum_every_frame': False,
        'reset_accum': True
    })
    g.set_dataset_settings({
    })
    g.set_render_every_frame(False)
    g.set_duration(0.0)
    g.set_duration(0.5)


def update_discrete(move_function, steps, time_per_step):
    move_function(0.0)
    for i in range(steps):
        g.set_duration(0)
        move_function((i + 1) / steps)
        g.set_duration(time_per_step)


def reset_accum():
    g.set_rendering_algorithm_settings({ 'reset_accum': True })


def replay():
    start_look_at = (0.0, 0.0, 0.0)
    start_position = (0.449055, 0.0598803, -0.055691)
    start_yaw = 3.0182
    start_pitch = -0.131569
    start_quat = g.convert_yaw_pitch_rad_to_quaternion(start_yaw, start_pitch)

    init_scene(start_look_at, start_position, start_yaw, start_pitch)
    g.set_duration(4)

    stop_position = (0.438433, 0.126672, 0.00829007)
    stop_yaw = -3.12269
    stop_pitch = -0.281213
    stop_quat = g.convert_yaw_pitch_rad_to_quaternion(stop_yaw, stop_pitch)

    def update(t):
        pos = tuple(start_position[i] + (stop_position[i] - start_position[i]) * t for i in range(3))
        g.set_camera_position(pos)
        quat = g.slerp(start_quat, stop_quat, t)
        g.set_camera_orientation_quaternion(quat)

    update_discrete(move_function=update, steps=2, time_per_step=3.0)

    g.set_rendering_algorithm_settings({
        'reset_accum_every_frame': False,
    })
    g.set_duration(0)
