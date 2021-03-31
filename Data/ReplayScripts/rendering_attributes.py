import g

def init_scene():
    g.set_duration(0)
    g.set_dataset('Femur (Vis2021, Teaser)')
    g.set_camera_checkpoint('TeaserA-2')
    g.set_rendering_algorithm_settings({
        'line_width': 0.005,
        'band_width': 0.008,
        'depth_cue_strength': 0.8
    })
    g.set_dataset_settings({
        'attribute': "Principal Stress Magnitude",
        'major_on': True,
        'medium_on': False,
        'minor_on': True,
        'major_lod': 0.3,
        'minor_lod': 0.5,
        'major_use_bands': True,
        'minor_use_bands': False,
        'thick_bands': True,
        'smoothed_bands': True,
        'use_principal_stress_direction_index': True,
    })
    g.set_transfer_functions(['qualitative-pale-lilac.xml', 'qualitative-emerald.xml', 'qualitative-ocher.xml'])
    g.set_transfer_functions_ranges([(0.0, 1.594), (0.0, 1.594), (0.0, 1.594)])
    #g.set_transfer_functions_ranges([(-0.06, 1.274), (-0.213, 0.157), (-0.96, 0.019)])
    g.set_duration(0.001)
    g.set_transfer_functions_ranges([(0.0, 1.594), (0.03, 1.594), (0.0, 1.594)])
    g.set_duration(6)

def use_principal_stress():
    g.set_duration(0)
    g.set_transfer_functions(['blues.xml', 'greens.xml', 'reds.xml'])
    #g.set_transfer_functions_ranges([(-0.071, 0.551), (-0.213, 0.157), (-0.665, 0.096)])
    #g.set_transfer_functions_ranges([(-0.06, 1.274), (-0.213, 0.157), (-0.96, 0.019)])
    g.set_duration(6)

def use_von_mises_stress():
    g.set_duration(0)
    g.set_dataset_settings({
        'attribute': "von Mises Stress",
        'use_principal_stress_direction_index': False
    })
    #g.set_transfer_function_range((0.031, 1.124))
    g.set_transfer_function_range((0.075, 1.236))
    g.set_duration(6)

def replay():
    init_scene()
    use_principal_stress()
    use_von_mises_stress()
