import numpy as np


def obj_twist(obj_velocity:np.ndarray=np.zeros(3),obj_rot:np.ndarray=np.zeros(3),obj_pos:np.ndarray=np.zeros(3)):
    v_O=obj_velocity+np.cross(obj_rot,obj_pos)
    twist=np.concatenate(v_O,obj_rot)
    return twist

def skew_matrix(vector):
    return np.array([[0, -vector[1], vector[2]],
                       [vector[1], 0, -vector[0]],
                       [-vector[2], vector[0], 0]])

def position_matrix(contact_pos,object_pos):
    skew=skew_matrix(contact_pos,object_pos)
    return np.block([[np.eye(3),np.zeros((3,3))],
                       [skew,np.eye(3)]])

def rotation_matrix(contact_rot):
    return np.block([[contact_rot,np.zeros((3,3))],
                       [np.zeros((3,3)),contact_rot]])

def grasp_matrix(contact_pos,object_pos,contact_rot):
    return rotation_matrix(contact_rot).transpose*position_matrix(contact_pos,object_pos).transpose

def z_matrix(contact_pos,joint_positions,joint_directions):
    z=np.zeros((6,4))
    for i in range(4):
        z[0:2,i]=skew_matrix(contact_pos-joint_positions[:,i])
        z[3:5,i]=joint_directions[:,i]
    return z

def jacobian(contact_pos, contact_rot,joint_positions,joint_directions):
    return rotation_matrix(contact_rot).transpose*z_matrix(contact_pos,joint_positions,joint_directions)






