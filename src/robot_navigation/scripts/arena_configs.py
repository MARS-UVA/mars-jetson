"""Hardcoded arena geometry, in meters in the map frame."""

ARENA_CONFIGS = {
    'ARENA_NASA': {
        # Excavation/obstacle border, halfway between the pillar's upper face
        # (y=2.75) and the top wall (y=5), facing into excavation (-X).
        'excavation_approach': (2.5, 3.875, 3.141592653589793),
        # PLACEHOLDER, not surveyed: the starting zone is not in this config.
        # Assumes the start-zone AprilTag is on the x=0 wall at y=1.0 (the
        # simulated tag-1 prior) and stands 1.5 m in front of it, facing it
        # (-X). Replace with the real start approach before relying on it.
        'start_approach': (1.5, 1.0, 3.141592653589793),
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
