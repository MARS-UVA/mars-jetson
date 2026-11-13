"""Tag-1 detector-frame poses in each bundled world's corner-based coordinates."""

# id x y z roll pitch yaw (meters/radians). These are apriltag_ros PnP
# frames: x image-right, y image-up, z out of the printed face.
# The observed +Z box face is 5 mm in front of the board's link origin.
NASA_ARENA_APRILTAG_PRIORS = '1 0.005 1 0.25 1.57079632679 0 1.57079632679'
NASA_COPY_APRILTAG_PRIORS = '1 0.0549999984 0.5 0.3400039816 -1.57 0 -1.57079632679'
UCF_ARENA_APRILTAG_PRIORS = '1 0.0049999984 0.435 0.3400039816 -1.57 0 -1.57079632679'

ARENA_APRILTAG_PRIORS = {
    'arena_nasa.world': NASA_ARENA_APRILTAG_PRIORS,
    'arena_nasa_copy.world': NASA_COPY_APRILTAG_PRIORS,
    'arena_nasa_copy2.world': NASA_COPY_APRILTAG_PRIORS,
    'arena_nasa_copy3.world': NASA_COPY_APRILTAG_PRIORS,
    'arena_nasa_craters.world': NASA_COPY_APRILTAG_PRIORS,
    'arena_ucf.world': UCF_ARENA_APRILTAG_PRIORS,
}
