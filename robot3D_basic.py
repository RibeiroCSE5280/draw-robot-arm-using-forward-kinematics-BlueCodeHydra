#!/usr/bin/env python
# coding: utf-8


from vedo import *
import ForwardKinematicsRobotArm as fk

def RotationMatrix(theta, axis_name):
    """ calculate single rotation of $theta$ matrix around x,y or z
        code from: https://programming-surgeon.com/en/euler-angle-python-en/
    input
        theta = rotation angle(degrees)
        axis_name = 'x', 'y' or 'z'
    output
        3x3 rotation matrix
    """

    c = np.cos(theta * np.pi / 180)
    s = np.sin(theta * np.pi / 180)
	
    if axis_name =='x':
        rotation_matrix = np.array([[1, 0,  0],
                                    [0, c, -s],
                                    [0, s,  c]])
    if axis_name =='y':
        rotation_matrix = np.array([[ c,  0, s],
                                    [ 0,  1, 0],
                                    [-s,  0, c]])
    elif axis_name =='z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s,  c, 0],
                                    [0,  0, 1]])
    return rotation_matrix


def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame
    Args:
      No input args
    Returns:
      F: vedo.mesh object (arrows for axis)
      
    """         
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alpha = 1
    
    
    # x-axis as an arrow  
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alpha)

    # y-axis as an arrow  
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alpha)

    # z-axis as an arrow  
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alpha)
    
    originDot = Sphere(pos=[0,0,0], 
                       c="black", 
                       r=0.10)


    # Combine the axes together to form a frame as a single mesh object 
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot
        
    return F


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

	

def main():
    # Set the limits of the graph x, y, and z ranges 
    axes = Axes(xrange=(0,20), yrange=(-2,10), zrange=(0,6))

    # Lengths of arm parts 
    L1 = 5   # Length of link 1
    L2 = 8   # Length of link 2
    L3 = 3   # Length of link 3 

    # Joint radius
    joint_radius = 0.4

    # Joint angles 
    phi1 = 30     # Rotation angle of part 1 in degrees
    phi2 = -10    # Rotation angle of part 2 in degrees
    phi3 = -10     # Rotation angle of part 3 in degrees

    # Matrix of Frame 1 (written w.r.t. Frame 0, which is the world frame) 
    R_01 = RotationMatrix(phi1, axis_name='z')   # Rotation matrix
    t_01 = np.array([[0], [0], [0]])             # Translation vector (origin for the first link)
    T_01 = getLocalFrameMatrix(R_01, t_01)       # Matrix of Frame 1 w.r.t. Frame 0
    
    # Create the first link and joint 
    joint0_sphere = Sphere(r=joint_radius).pos(0, 0, 0).color("gray").alpha(.8)
    link1_mesh = Cylinder(r=joint_radius, height=L1, pos=(L1/2, 0, 0), c="yellow", alpha=.8, axis=(1,0,0))
    joint1_sphere = Sphere(r=joint_radius).pos(L1, 0, 0).color("gray").alpha(.8)
    Frame1 = createCoordinateFrameMesh() + joint0_sphere + link1_mesh + joint1_sphere
    Frame1.apply_transform(T_01)

    # Matrix of Frame 2 (written w.r.t. Frame 1) 
    R_12 = RotationMatrix(phi2, axis_name='z')   # Rotation matrix
    t_12 = np.array([[L1], [0], [0]])            # Translation vector
    T_12 = getLocalFrameMatrix(R_12, t_12)       # Matrix of Frame 2 w.r.t. Frame 1
    T_02 = T_01 @ T_12                           # Matrix of Frame 2 w.r.t. Frame 0

    # Create the second link and joint
    link2_mesh = Cylinder(r=0.4, height=L2, pos=(L2/2, 0, 0), c="red", alpha=.8, axis=(1,0,0))
    joint2_sphere = Sphere(r=0.4).pos(L2, 0, 0).color("gray").alpha(.8)
    Frame2 = createCoordinateFrameMesh() + link2_mesh + joint2_sphere
    Frame2.apply_transform(T_02)

    # Matrix of Frame 3 (written w.r.t. Frame 2) 
    R_23 = RotationMatrix(phi3, axis_name='z')   # Rotation matrix
    t_23 = np.array([[L2], [0], [0]])            # Translation vector
    T_23 = getLocalFrameMatrix(R_23, t_23)       # Matrix of Frame 3 w.r.t. Frame 2
    T_03 = T_02 @ T_23                           # Matrix of Frame 3 w.r.t. Frame 0

    # Create the third link and joint
    link3_mesh = Cylinder(r=0.4, height=L3, pos=(L3/2, 0, 0), c="blue", alpha=.8, axis=(1,0,0))
    joint3_sphere = Sphere(r=0.4).pos(L3, 0, 0).color("gray").alpha(.8)
    Frame3 = createCoordinateFrameMesh() + link3_mesh + joint3_sphere
    Frame3.apply_transform(T_03)

    # Show everything 
    show([Frame1, Frame2, Frame3], axes, viewup="z").close()

if __name__ == '__main__':
    main()




