import numpy as np

def RotationMatrix(theta):
    """Calculates the rotation matrix for a rotation around the z-axis by theta degrees."""
    theta = np.radians(theta)  # Convert to radians
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]])

def TranslationMatrix(d):
    """Creates a translation matrix for a translation along the x-axis by d units."""
    return np.array([[1, 0, 0, d],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def forward_kinematics(Phi, L1, L2, L3, L4):
    """
    Calculate the local-to-global frame matrices and the location of the end-effector.
    """
    # Start with the base frame
    T_0 = np.eye(4)
    
    # Apply translation then rotation for each joint
    T_01 = T_0 @ TranslationMatrix(L1) @ RotationMatrix(Phi[0])
    T_02 = T_01 @ TranslationMatrix(L2) @ RotationMatrix(Phi[1])
    T_03 = T_02 @ TranslationMatrix(L3) @ RotationMatrix(Phi[2])
    T_04 = T_03 @ TranslationMatrix(L4)  # L4 is the offset for the end-effector
    
    # The end-effector's position is the translation part of the T_04 matrix
    e = T_04[:3, 3]

    return T_01, T_02, T_03, T_04, e
