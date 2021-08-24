import g

def init_scene():
    g.set_duration(0)
    g.set_dataset('Arched Bridge 3D (TVCG01, Fig 6, Fig 9)')
    #g.set_dataset('KittenHex (Vis2021, results)')
    g.set_camera_checkpoint('Front')
    g.set_rendering_algorithm_settings({
        'line_width': 0.0022,
        'band_width': 0.016,
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
    g.set_transfer_functions(['qualitative-ocher.xml', 'qualitative-emerald.xml', 'qualitative-pale-lilac.xml'])
    g.set_duration(6)

def mode0():
    g.set_duration(0)
    g.set_dataset_settings({
        'major_lod': 0.4
    })
    g.set_duration(6)

def mode1():
    g.set_duration(0)
    g.set_dataset_settings({
        'major_on': False
    })
    g.set_duration(6)

def mode2():
    g.set_duration(0)
    g.set_dataset_settings({
        'minor_lod': 0.4
    })
    g.set_duration(6)

def mode3():
    g.set_duration(0)
    g.set_dataset_settings({
        'major_on': True,
        'major_lod': 1.0,
    })
    g.set_duration(6)

def mode4():
    g.set_duration(0)
    g.set_dataset_settings({
        'minor_use_bands': True
    })
    g.set_duration(6)

def replay():
    init_scene()
    mode0()
    mode1()
    mode2()
    mode3()
    mode4()
