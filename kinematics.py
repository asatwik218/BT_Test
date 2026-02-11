import numpy as np

def Cartesian_Pose_To_Hom_Mat(pose):
    """
    Convert a Doosan Cartesian pose (XYZABC) with ZY'Z'' intrinsic rotations
    into a homogeneous transformation matrix.

    Parameters:
        pose (list/tuple): [X, Y, Z, A, B, C]
                           - X, Y, Z: position (same units in output)
                           - A, B, C: angles in degrees (ZY'Z'' intrinsic convention)

    Returns:
        np.ndarray: 4x4 homogeneous transformation matrix
    """
    # Extract position and orientation
    X, Y, Z, A_deg, B_deg, C_deg = pose
    
    # Convert degrees to radians
    A = np.deg2rad(A_deg)
    B = np.deg2rad(B_deg)
    C = np.deg2rad(C_deg)
    
    # Rotation matrices for intrinsic Z-Y'-Z''
    Rz1 = np.array([
        [np.cos(A), -np.sin(A), 0],
        [np.sin(A),  np.cos(A), 0],
        [0, 0, 1]
    ])
    
    Ry = np.array([
        [np.cos(B), 0, np.sin(B)],
        [0, 1, 0],
        [-np.sin(B), 0, np.cos(B)]
    ])
    
    Rz2 = np.array([
        [np.cos(C), -np.sin(C), 0],
        [np.sin(C),  np.cos(C), 0],
        [0, 0, 1]
    ])
    
    # Intrinsic Z-Y'-Z'' rotation = Rz1 * Ry * Rz2
    R = Rz1 @ Ry @ Rz2
    
    # Homogeneous transformation matrix
    H = np.eye(4)
    H[0:3, 0:3] = R
    H[0:3, 3] = [X, Y, Z]
    
    return H


def Translation_To_Hom_Mat(dx, dy, dz):
    """
    Create a homogeneous transformation matrix for a pure translation.

    Parameters:
        dx (float): displacement along X axis
        dy (float): displacement along Y axis
        dz (float): displacement along Z axis

    Returns:
        np.ndarray: 4x4 homogeneous transformation matrix
    """
    H = np.eye(4)
    H[0:3, 3] = [dx, dy, dz]
    return H


def X_Rotation_To_Hom_Mat(theta_deg):
    """
    Homogeneous transformation for rotation about X axis.
    """
    theta = np.deg2rad(theta_deg)
    Rx = np.array([
        [1, 0,             0,              0],
        [0, np.cos(theta), -np.sin(theta), 0],
        [0, np.sin(theta),  np.cos(theta), 0],
        [0, 0,              0,              1]
    ])
    return Rx


def Y_Rotation_To_Hom_Mat(theta_deg):
    """
    Homogeneous transformation for rotation about Y axis.
    """
    theta = np.deg2rad(theta_deg)
    Ry = np.array([
        [ np.cos(theta), 0, np.sin(theta), 0],
        [ 0,             1, 0,             0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [ 0,             0, 0,             1]
    ])
    return Ry


def Z_Rotation_To_Hom_Mat(theta_deg):
    """
    Create a homogeneous transformation matrix for rotation
    of a coordinate frame about the Z axis.

    Parameters:
        theta_deg (float): rotation angle in degrees (counterclockwise)

    Returns:
        np.ndarray: 4x4 homogeneous transformation matrix
    """
    theta = np.deg2rad(theta_deg)

    Rz = np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta),  np.cos(theta), 0, 0],
        [0,              0,             1, 0],
        [0,              0,             0, 1]
    ])

    return Rz


