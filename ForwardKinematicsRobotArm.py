import numpy as np

def RotationMatrix(theta):
    """Calculates the rotation matrix for a rotation around the z-axis by theta degrees."""
    theta = np.radians(theta)  # Convert to radians
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])

def forward_kinematics(Phi, L1, L2, L3, L4):
    """
    Calculates the transformation matrices and the location of the end-effector.
    
    Args:
        Phi (4x1 ndarray): Array containing the four joint angles.
        L1, L2, L3, L4 (float): Lengths of the parts of the robot arm.
    
    Returns:
        T_01, T_02, T_03, T_04 (4x4 ndarrays): Transformation matrices for each frame.
        e (3x1 ndarray): 3D coordinates, the location of the end-effector in space.
    """
    T_0 = np.eye(4)  # Base frame (world frame)
    transformations = [T_0]  # Initialize with the base frame

    # Accumulate transformations for each segment
    lengths = [0, L1, L2, L3]  # The first segment (base) has no length
    for i, (phi, L) in enumerate(zip(Phi, lengths)):
        R = RotationMatrix(phi)
        T = np.eye(4)
        T[:3, :3] = R
        if i > 0:  # No translation for the base frame
            T[:3, 3] = [lengths[i-1], 0, 0]  # Translate along x of the previous frame
        transformations.append(T @ transformations[i])  # Update transformation

    # Final transformation matrix and end-effector position
    T_04 = transformations[-1] @ np.array([[1, 0, 0, L4],
                                           [0, 1, 0, 0],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]])
    e = T_04[:3, 3]  # Extract translation component for end-effector position

    return transformations[1], transformations[2], transformations[3], T_04, e
