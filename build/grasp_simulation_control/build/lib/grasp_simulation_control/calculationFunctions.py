import numpy as np

def skew_symmetric(v):
    """Create skew-symmetric matrix from 3D vector"""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def contact_selection_matrix(contact_type):
    """
    Return selection matrix B based on contact type
    SF: Soft Finger - can transmit forces and one torque component
    HF: Hard Finger - can transmit only forces
    FF: Frictionless Finger - can transmit only normal force
    """
    if contact_type == "SF":  # Soft Finger
        return np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    elif contact_type == "HF":  # Hard Finger
        return np.array([[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1],
                         [0, 0, 0]])
    elif contact_type == "FF":  # Frictionless Finger
        return np.array([[0],
                         [0],
                         [1],
                         [0]])
    else:
        raise ValueError(f"Unknown contact type: {contact_type}")

def grasp_matrix_transposed_and_jacobian(contact_positions, contact_orientations, 
                                         joint_positions, joint_directions, 
                                         object_position, contact_type="SF"):
    """
    Calculate the transposed grasp matrix G^T and hand Jacobian J
    
    Args:
        contact_positions: array of contact positions (n_contacts x 3)
        contact_orientations: array of contact frame orientations (n_contacts x 3 x 3)
        joint_positions: array of joint positions for each finger (n_contacts x 3 x n_joints)
        joint_directions: array of joint axes for each finger (n_contacts x 3 x n_joints)
        object_position: position of object center (3,)
        contact_type: "SF", "HF", or "FF"
    
    Returns:
        G_t: Transposed grasp matrix
        J: Hand Jacobian
    """
    n_contacts = len(contact_positions)
    B = contact_selection_matrix(contact_type)
    
    # Determine dimensions based on contact type
    if contact_type == "SF":
        wrench_dim = 4
    elif contact_type == "HF":
        wrench_dim = 3
    elif contact_type == "FF":
        wrench_dim = 1
    
    # Initialize G^T matrix (6 x (n_contacts * wrench_dim))
    G_t = np.zeros((6, n_contacts * wrench_dim))
    
    # Build G^T for each contact
    for i in range(n_contacts):
        # Contact position relative to object center
        r_i = contact_positions[i] - object_position
        
        # Contact frame rotation matrix
        R_i = contact_orientations[i]
        
        # Build the partial grasp matrix for this contact
        # Upper part: R_i
        # Lower part: S(r_i) @ R_i
        upper = R_i
        lower = skew_symmetric(r_i) @ R_i
        
        # Combine upper and lower parts
        W_i = np.vstack([upper, lower])
        
        # Apply contact model selection
        G_i = W_i @ B
        
        # Insert into full G^T matrix
        start_col = i * wrench_dim
        end_col = start_col + wrench_dim
        G_t[:, start_col:end_col] = G_i
    
    # Calculate hand Jacobian J
    # J maps joint velocities to contact point velocities
    # Size: (n_contacts * 3) x (total_n_joints)
    
    # Count total joints (assuming 4 joints per finger)
    n_joints_per_finger = joint_positions.shape[2]
    total_joints = n_contacts * n_joints_per_finger
    
    J = np.zeros((n_contacts * 3, total_joints))
    
    for i in range(n_contacts):
        # For each contact/finger
        for j in range(n_joints_per_finger):
            # Joint position and axis
            p_j = joint_positions[i, :, j]
            a_j = joint_directions[i, :, j]
            
            # Vector from joint to contact point
            r_jc = contact_positions[i] - p_j
            
            # Linear velocity contribution: a_j x r_jc
            if np.linalg.norm(a_j) > 0:  # Check if joint axis is valid
                v_linear = np.cross(a_j, r_jc)
            else:
                v_linear = np.zeros(3)
            
            # Insert into Jacobian
            row_start = i * 3
            row_end = row_start + 3
            col = i * n_joints_per_finger + j
            
            J[row_start:row_end, col] = v_linear
    
    return G_t, J

def compute_grasp_quality(G_t, contact_type="SF"):
    """
    Compute grasp quality metrics
    
    Args:
        G_t: Transposed grasp matrix
        contact_type: Contact model type
    
    Returns:
        dict: Dictionary containing various grasp quality metrics
    """
    # Compute G from G^T
    G = G_t.T
    
    # Force closure test: Check if G has full row rank
    rank = np.linalg.matrix_rank(G)
    is_force_closure = rank == 6
    
    # Compute smallest singular value (grasp isotropy)
    _, s, _ = np.linalg.svd(G)
    min_singular_value = np.min(s)
    
    # Compute condition number
    condition_number = np.max(s) / np.min(s) if np.min(s) > 1e-10 else np.inf
    
    # Volume of grasp wrench space (epsilon quality)
    # This is proportional to the product of singular values
    wrench_space_volume = np.prod(s)
    
    return {
        'force_closure': is_force_closure,
        'min_singular_value': min_singular_value,
        'condition_number': condition_number,
        'wrench_space_volume': wrench_space_volume,
        'rank': rank
    }

def compute_contact_forces(G_t, external_wrench, method='pinv'):
    """
    Compute contact forces to balance external wrench
    
    Args:
        G_t: Transposed grasp matrix
        external_wrench: External wrench on object (6,)
        method: 'pinv' for pseudo-inverse, 'opt' for optimization
    
    Returns:
        contact_forces: Forces at contacts
    """
    if method == 'pinv':
        # Use pseudo-inverse (minimum norm solution)
        G_t_pinv = np.linalg.pinv(G_t)
        contact_forces = -G_t_pinv @ external_wrench
    elif method == 'opt':
        # Could implement quadratic programming here
        # For now, use pseudo-inverse
        contact_forces = compute_contact_forces(G_t, external_wrench, 'pinv')
    
    return contact_forces

def compute_joint_torques(J, contact_forces):
    """
    Compute required joint torques from contact forces
    
    Args:
        J: Hand Jacobian
        contact_forces: Desired contact forces
    
    Returns:
        joint_torques: Required joint torques
    """
    # Expand contact forces to include all components (assuming 3D forces)
    n_contacts = len(contact_forces) // 3  # Assuming 3D force vectors
    force_vector = contact_forces[:n_contacts * 3]  # Take only force components
    
    # tau = J^T * F
    joint_torques = J.T @ force_vector
    
    return joint_torques