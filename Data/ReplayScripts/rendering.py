import g

def init_scene():
    g.set_duration(0)
    g.set_dataset('Cantilever (Vis2021)')
    g.set_camera_checkpoint('Overview')
    g.set_rendering_algorithm_settings({
        'line_width': 0.007,
        'band_width': 0.005,
        'depth_cue_strength': 0.0
    })
    g.set_dataset_settings({
        'attribute': "Principal Stress",
        'major_on': True,
        'medium_on': False,
        'minor_on': True,
        'major_lod': 0.5,
        'minor_lod': 0.5,
        'major_use_bands': True,
        'minor_use_bands': False,
        'thick_bands': False,
        'smoothed_bands': True,
        'use_principal_stress_direction_index': True,
    })
    g.set_transfer_functions(['qualitative-pale-lilac.xml', 'qualitative-emerald.xml', 'qualitative-ocher.xml'])

def elliptic_tubes_off_depth_cues_off():
    g.set_duration(6)

def elliptic_tubes_off_depth_cues_on():
    g.set_duration(0)
    g.set_rendering_algorithm_settings({
        'depth_cue_strength': 1.0
    })
    g.set_duration(6)

def elliptic_tubes_on_depth_cues_on():
    g.set_duration(0)
    g.set_dataset_settings({
        'thick_bands': True
    })
    g.set_duration(6)

#def use_principal_stress():
#    g.set_duration(0)
#    g.set_transfer_functions(['blues.xml', 'greens.xml', 'reds.xml'])
#    g.set_duration(6)
#
#def use_von_mises_stress():
#    g.set_duration(0)
#    g.set_dataset_settings({
#        'attribute': "von Mises Stress",
#        'use_principal_stress_direction_index': False
#    })
#    g.set_duration(6)

def replay():
    init_scene()
    elliptic_tubes_off_depth_cues_off()
    elliptic_tubes_off_depth_cues_on()
    elliptic_tubes_on_depth_cues_on()
    #use_principal_stress()
    #use_von_mises_stress()
