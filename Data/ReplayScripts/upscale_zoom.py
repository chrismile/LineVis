import math
from modules.campath import camera_path_circle
import g

def init_scene(start_look_at, start_position, start_yaw, start_pitch):
    g.set_duration(0)
    #g.set_renderer('Deferred Opaque')
    g.set_dataset('Cantilever (Vis2021)')
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
    start_position = (0.0, 0.0, 1.62306)
    start_yaw = -1.5708
    start_pitch = 0.0

    init_scene(start_look_at, start_position, start_yaw, start_pitch)
    g.set_duration(4)

    closeup_position = (0.0, 0.0, 0.456437)

    def update_pos(t):
        pos = tuple(start_position[i] + (closeup_position[i] - start_position[i]) * t for i in range(3))
        g.set_camera_position(pos)

    update_discrete(move_function=update_pos, steps=4, time_per_step=0.5)
    #g.set_duration(2)
    #g.set_camera_position(closeup_position)
    #g.set_duration(2)

    g.set_rendering_algorithm_settings({
        'reset_accum_every_frame': False,
    })
    g.set_duration(0)
