#!/usr/bin/env python 

import numpy as np
from scipy.spatial.transform import Rotation
import motion_utils as utils

class RotationTrajectory():
    """"
    trajectory containing sequence of rotations from a fixed frame, angular velocity, and angular acceleration
    """
    def __init__(self, rot, dur=None, t=None):
        if dur is None and t is None:
            raise ValueError(self.__class__.__name__+': Must provide either duration or array of time stamps')

        n = len(rot)
        dt = dur/(n-1)
        _rot = self.init_rotation(rot)
        self.R = _rot.as_matrix()
        self.q = _rot.as_quat()
        self.axes = None
        self.angles = None
        self.n = n
        self.dur = dur
        self.dt = dt

        angvel, body_angvel = utils.angvel_from_rotations(self.R,dt)
        self.angvel = angvel
        self.body_angvel = body_angvel



    def init_rotation(self,rot):
        """
        Initialize a scipy Rotation object based on dimensions of given the input
        return rotation object
        """
        N = len(rot)
        dim = rot.shape
        if dim == (4,) or dim == (N,4):
            out = Rotation.from_quat(rot)

        elif dim == (3,3) or dim == (N,3,3):
            out = Rotation.from_matrix(rot)

        elif dim == (3,) or dim == (N,3):
            out = Rotation.from_euler('xyz', rot)
        
        else:
            raise ValueError('Invalid rotation input with shape {}'.format(dim))
        return out

    def relative(self,a,b):
        """
        Compute relative rotation between two rotations a and b (frame b expressed in frame a)
        params
            a: int, index of first rotation
            b: int, index of second rotation
        returns
            scipy Rotation object 
        """
        Ra = self.R[a]
        Rb = self.R[b]
        return Rotation.from_matrix(Ra.T@Rb)
    
    def relative_matrix(self,a,b):
        return self.relative(a,b).as_matrix()
    
    def relative_quat(self,a,b):
        return self.relative(a,b).as_quat()
    
    def relative_axis(self,a,b):
        return utils.unit(self.relative(a,b).as_rotvec())
    
    def relative_angle(self,a,b):
        return np.linalg.norm(self.relative_axis(a,b),axis=1)
    

        
 
