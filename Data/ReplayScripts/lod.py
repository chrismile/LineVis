import g

def init_scene():
    g.set_duration(0)
    g.set_dataset('Kitten (Vis2021, Fig. 6)')
    g.set_camera_checkpoint('Overview')
    g.set_rendering_algorithm_settings({
        'line_width': 0.005,
        'band_width': 0.020,
        'depth_cue_strength': 0.8
    })
    g.set_dataset_settings({
        'attribute': "Principal Stress",
        'major_on': True,
        'medium_on': False,
        'minor_on': True,
        'major_lod': 1.0,
        'medium_lod': 1.0,
        'minor_lod': 1.0,
        'major_use_bands': False,
        'medium_use_bands': False,
        'minor_use_bands': False,
        'thick_bands': True,
        'smoothed_bands': True,
        'use_principal_stress_direction_index': True,
    })
    g.set_transfer_functions(['qualitative-pale-lilac.xml', 'qualitative-emerald.xml', 'qualitative-ocher.xml'])
    g.set_duration(6)

def bottom0():
    g.set_duration(0)
    g.set_dataset('Kitten (Vis2021, Fig. 6, bottom0)')
    g.set_duration(6)

def bottom1():
    g.set_duration(0)
    g.set_dataset('Kitten (Vis2021, Fig. 6, bottom1)')
    g.set_duration(6)

def bottom2():
    g.set_duration(0)
    g.set_dataset('Kitten (Vis2021, Fig. 6, bottom2)')
    g.set_duration(6)

def replay():
    init_scene()
    bottom0()
    bottom1()
    bottom2()
