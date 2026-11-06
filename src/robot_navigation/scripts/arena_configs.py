"""Hardcoded arena geometry, in meters in the map frame."""

ARENA_CONFIGS = {
    'ARENA_NASA': {
        'zones': {
            'Excavation zone': [(0.0, 0.0), (2.5, 0.0), (2.5, 5.0), (0.0, 5.0)],
            'Obstacle zone': [
                (2.5, 0.0), (3.88, 0.0), (3.88, 1.5),
                (6.88, 1.5), (6.88, 5.0), (2.5, 5.0),
            ],
            'Construction zone': [(3.88, 0.0), (6.88, 0.0), (6.88, 1.5), (3.88, 1.5)],
        },
        # The exact berm location is not specified in the arena diagram.
        # For now, center the 2.2m x 0.9m berm within the construction zone.
        'berm': {'center': (5.38, 0.75), 'dimensions': (2.2, 0.9)},
    },
}
