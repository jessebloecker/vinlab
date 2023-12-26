#!/usr/bin/env python 

import numpy as np
from scipy.spatial.transform import Rotation
import motion_utils as utils

class RotationTrajectory():
    """"
    trajectory containing sequence of rotations from a fixed frame, angular velocity, and angular acceleration
    """
    def __init__(self, rot, t=None, dur=None):

        n = len(rot)
        _t, _dur = utils.init_trajectory_timestamps(n,t,dur)
        dt = _dur/(n-1)
        
        _rot = utils.as_scipy_rotation(rot)
        R = _rot.as_matrix()
        angvel, body_angvel = utils.angvel_from_rotations(R,dt)

        self.R = _rot.as_matrix()
        self.angvel = angvel
        self.body_angvel = body_angvel
        self.n = n
        self.dur = _dur
        self.t = _t
        self.dt = dt
        self.rate = 1./dt

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
    

        
 
