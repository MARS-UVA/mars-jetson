import numpy as np
from tf_parser_msgs.msg import AprilTagPositions, AbsolutePosition

def calculate_absolute_position(self, positions: AprilTagPositions):

    camera_positions = []
    camera_quaternions = []

    for position in positions.positions:

        if position.tag_name not in self.known_tags:
            continue

        known = self.known_tags[position.tag_name]

        # ---------------------------------------------------------
        # Known tag pose in world
        # T_world_tag
        # ---------------------------------------------------------
        p_world_tag = np.array([
            known.x,
            known.y,
            known.z
        ])

        q_world_tag = np.array([
            known.qx,
            known.qy,
            known.qz,
            known.qw
        ])

        R_world_tag = quaternion_to_rotation_matrix(
            q_world_tag
        )

        # ---------------------------------------------------------
        # TF gives:
        #
        # lookup_transform('odom', tag)
        #
        # = T_odom_tag
        #
        # tag coordinates -> odom coordinates
        # ---------------------------------------------------------

        p_odom_tag = np.array([
            position.x,
            position.y,
            position.z
        ])

        q_odom_tag = np.array([
            position.qx,
            position.qy,
            position.qz,
            position.qw
        ])

        R_odom_tag = quaternion_to_rotation_matrix(
            q_odom_tag
        )

        # ---------------------------------------------------------
        # Invert T_odom_tag
        #
        # T_tag_odom
        # ---------------------------------------------------------

        R_tag_odom = R_odom_tag.T

        p_tag_odom = -R_tag_odom @ p_odom_tag

        # ---------------------------------------------------------
        # Compose:
        #
        # T_world_odom =
        #     T_world_tag * T_tag_odom
        # ---------------------------------------------------------

        p_world_odom = (
            p_world_tag
            + R_world_tag @ p_tag_odom
        )

        R_world_odom = (
            R_world_tag @ R_tag_odom
        )

        q_world_odom = rotation_matrix_to_quaternion(
            R_world_odom
        )

        camera_positions.append(p_world_odom)
        camera_quaternions.append(q_world_odom)

    if not camera_positions:
        return None

    # ---------------------------------------------------------
    # Average position
    # ---------------------------------------------------------

    positions_array = np.array(camera_positions)

    mean_position = np.mean(
        positions_array,
        axis=0
    )

    std_position = np.std(
        positions_array,
        axis=0
    )

    # ---------------------------------------------------------
    # Average quaternion
    # ---------------------------------------------------------

    mean_quaternion = average_quaternions(
        camera_quaternions
    )

    absolute_position = AbsolutePosition()

    absolute_position.x = float(mean_position[0])
    absolute_position.y = float(mean_position[1])
    absolute_position.z = float(mean_position[2])

    absolute_position.qx = float(mean_quaternion[0])
    absolute_position.qy = float(mean_quaternion[1])
    absolute_position.qz = float(mean_quaternion[2])
    absolute_position.qw = float(mean_quaternion[3])

    absolute_position.std_x = float(std_position[0])
    absolute_position.std_y = float(std_position[1])
    absolute_position.std_z = float(std_position[2])

    return absolute_position

def quaternion_to_rotation_matrix(q):

    qx, qy, qz, qw = q

    # Normalize quaternion
    norm = np.linalg.norm(q)

    if norm == 0:
        raise ValueError("Invalid zero-length quaternion")

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    return np.array([
        [
            1 - 2 * (qy*qy + qz*qz),
            2 * (qx*qy - qz*qw),
            2 * (qx*qz + qy*qw)
        ],
        [
            2 * (qx*qy + qz*qw),
            1 - 2 * (qx*qx + qz*qz),
            2 * (qy*qz - qx*qw)
        ],
        [
            2 * (qx*qz - qy*qw),
            2 * (qy*qz + qx*qw),
            1 - 2 * (qx*qx + qy*qy)
        ]
    ])


def rotation_matrix_to_quaternion(R):

    # Markley-style conversion from rotation matrix
    # to quaternion [qx, qy, qz, qw].

    trace = np.trace(R)

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)

        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s

    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:

        s = 2.0 * np.sqrt(
            1.0 + R[0, 0] - R[1, 1] - R[2, 2]
        )

        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s

    elif R[1, 1] > R[2, 2]:

        s = 2.0 * np.sqrt(
            1.0 + R[1, 1] - R[0, 0] - R[2, 2]
        )

        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s

    else:

        s = 2.0 * np.sqrt(
            1.0 + R[2, 2] - R[0, 0] - R[1, 1]
        )

        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw])

    return q / np.linalg.norm(q)


def average_quaternions(quaternions):

    """
    Average quaternions using the Markley method.

    Input:
        list of [qx, qy, qz, qw]

    Output:
        [qx, qy, qz, qw]
    """

    A = np.zeros((4, 4))

    for q in quaternions:

        q = np.asarray(q, dtype=float)

        # Normalize
        q /= np.linalg.norm(q)

        # q and -q represent the same rotation, so
        # averaging the raw components would be incorrect.
        A += np.outer(q, q)

    A /= len(quaternions)

    eigenvalues, eigenvectors = np.linalg.eigh(A)

    # Eigenvector corresponding to largest eigenvalue
    q = eigenvectors[:, np.argmax(eigenvalues)]

    # Normalize
    q /= np.linalg.norm(q)

    return q
