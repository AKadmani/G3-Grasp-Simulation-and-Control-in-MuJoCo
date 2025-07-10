import numpy as np
from scipy.linalg import block_diag

# Compute the twist (velocity and angular velocity) of an object
def obj_twist(
    obj_velocity: np.ndarray = np.zeros(3),
    obj_rot: np.ndarray = np.zeros(3),
    obj_pos: np.ndarray = np.zeros(3)
) -> np.ndarray:
    v_O = obj_velocity + np.cross(obj_rot, obj_pos)
    twist = np.concatenate(v_O, obj_rot)
    return twist

# Return the skew-symmetric matrix of a vector
def skew_matrix(vector: np.ndarray) -> np.ndarray:
    return np.array([[0, -vector[1], vector[2]],
                       [vector[1], 0, -vector[0]],
                       [-vector[2], vector[0], 0]])

# Build the position matrix for grasp calculations
def position_matrix(contact_pos: np.ndarray, object_pos: np.ndarray) -> np.ndarray:
    skew = skew_matrix(contact_pos-object_pos)
    return np.block([[np.eye(3), np.zeros((3, 3))],
                       [skew, np.eye(3)]])

# Build the rotation matrix for grasp calculations
def rotation_matrix(contact_rot: np.ndarray) -> np.ndarray:
    return np.block([[contact_rot, np.zeros((3, 3))],
                       [np.zeros((3, 3)), contact_rot]])

# Compute the partial grasp matrix (transposed) for a contact point
def partial_grasp_matrix_transposed(
    contact_pos: np.ndarray,
    object_pos: np.ndarray,
    contact_rot: np.ndarray
) -> np.ndarray:
    return rotation_matrix(contact_rot).transpose() @ position_matrix(contact_pos, object_pos).transpose()

# Build the Z matrix for a finger, relating joint velocities to contact velocities
def z_matrix(
    contact_pos: np.ndarray,
    joint_positions: np.ndarray,
    joint_directions: np.ndarray
) -> np.ndarray:
    z = np.zeros((6, 4))
    for i in range(4):
        # Top 3 rows: effect of joint axis on position; bottom 3: joint axis direction
        z[0:3, i] = skew_matrix(contact_pos - joint_positions[:, i]) @ joint_directions[:, i]
        z[3:6, i] = joint_directions[:, i]
    return z

# Compute the partial Jacobian for a contact point
def partial_jacobian(
    contact_pos: np.ndarray,
    contact_rot: np.ndarray,
    joint_positions: np.ndarray,
    joint_directions: np.ndarray
) -> np.ndarray:
    return rotation_matrix(contact_rot).transpose() @ z_matrix(contact_pos, joint_positions, joint_directions)

# Return the selection matrix for a given contact type (SF, HF, FF)
def selection_matrix(contact_type: str) -> np.ndarray:
    if contact_type == "SF":
        # Soft finger: 4 DoF
        return np.array([[1, 0, 0, 0, 0, 0],
                          [0, 1, 0, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 1, 0, 0]])
    
    elif contact_type == "HF":
        # Hard finger: 3 DoF
        return np.array([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])
    
    elif contact_type == "FF":
        # Frictionless finger: 1 DoF
        return np.array([[1, 0, 0]])
    else:
        raise ValueError("Invalid contact type. Choose from 'SF', 'HF', or 'FF'.")

# Compute the full grasp matrix (transposed) and Jacobian for all fingers
def grasp_matrix_transposed_and_jacobian(
    contact_pos_all: np.ndarray,
    contact_rot_all: np.ndarray,
    joint_positions_all: np.ndarray,
    joint_directions: np.ndarray,
    object_position: np.ndarray,
    contact_type: str
) -> tuple[np.ndarray, np.ndarray]:
    for i in range(contact_pos_all.shape[0]):
        if i == 0:
            B = selection_matrix(contact_type)
            # Grasp matrix and Jacobian for the first finger
            G_t = B @ partial_grasp_matrix_transposed(contact_pos_all[i, :], object_position, contact_rot_all[i, :, :])
            J = B @ partial_jacobian(contact_pos_all[i, :], contact_rot_all[i, :], joint_positions_all[i, :, :], joint_directions[i, :, :])
        else:
            B = selection_matrix(contact_type)
            # Block diagonal concatenation for additional fingers
            G_t = block_diag(G_t, B @ partial_grasp_matrix_transposed(contact_pos_all[i, :], object_position, contact_rot_all[i, :, :]))
            J = block_diag(J, B @ partial_jacobian(contact_pos_all[i, :], contact_rot_all[i, :], joint_positions_all[i, :, :], joint_directions[i, :, :]))
    return G_t, J