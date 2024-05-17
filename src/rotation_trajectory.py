#!/usr/bin/env python 

import numpy as np
from scipy.spatial.transform import Rotation
from motion_utils import init_trajectory_timestamps, angvel_from_rotations, time_derivative
from geometry_utils import as_scipy_rotation
from row_vector_array import RowVectorArray

class RotationTrajectory():
    """"
    trajectory containing sequence of rotations from a fixed frame, angular velocity, and angular acceleration
    """
    def __init__(self, rot, t=None, dur=None):

        n = len(rot)
        _t, _dur = init_trajectory_timestamps(n,t,dur)
        dt = _dur/(n-1)
        
        _rot = as_scipy_rotation(rot)
        R = _rot.as_matrix()
        angvel, body_angvel = angvel_from_rotations(R,dt)
        angacc = time_derivative(1,angvel,dt)

        self.rot = _rot
        self.angvel = RowVectorArray(angvel)
        self.body_angvel = RowVectorArray(body_angvel)
        self.angacc = RowVectorArray(angacc)
        self.body_angacc = RowVectorArray((R.swapaxes(1,2)@angacc.reshape(n,3,1)).reshape(n,3))
        
        self.n = n
        self.dur = _dur
        self.t = _t
        self.dt = dt
        self.rate = 1./dt

    def relative(self,a,b):
        """
        Compute relative rotation between two rotations 'a' and 'b'.
        The resulting rotation matrix is frame 'b' expressed in frame 'a'.
        params
            a: int, index of first rotation
            b: int, index of second rotation
        returns
            rot: scipy Rotation object, relative rotation 
        """
        R = self.rot.as_matrix()
        Ra = R[a]
        Rb = R[b]
        return as_scipy_rotation(Ra.T@Rb)
    
