#!/usr/bin/env python 

import numpy as np
from motion_utils import init_trajectory_timestamps, time_derivative
from row_vector_array import RowVectorArray
from config_utils import check_keys, ConfigurationError

class TranslationTrajectory():
    """"
    trajectory containing position, velocity, and acceleration
    """
    def __init__(self, pos, t=None, dur=None, vel=None, acc=None, trajectory_type=None):
        n = len(pos)
        _t, _dur = init_trajectory_timestamps(n,t,dur)
        dt = _dur/(n-1)

        numerical_vel = time_derivative(1,pos,dt)
        numerical_acc = time_derivative(1,numerical_vel,dt)
        _vel = numerical_vel if (vel is None) else vel
        _acc = numerical_acc if (acc is None) else acc

        self.pos = RowVectorArray(pos)
        self.vel = RowVectorArray(_vel)
        self.acc = RowVectorArray(_acc)
        self.numerical_vel = RowVectorArray(numerical_vel)
        self.numerical_acc = RowVectorArray(numerical_acc)
        self.n = n
        self.dur = _dur
        self.t = _t
        self.dt = dt
        self.rate = 1./dt
        self.type = trajectory_type
        self.centroid = np.mean(pos, axis=0)

    @classmethod
    def config(cls,config):
        config, config_mode  = check_keys(config, 'translation', context='trajectory')
        
        if config_mode == {'bspline'}:
            return BSpline.config(config['bspline'])
        elif config_mode == {'constant'}:
            return config #return the config because it's dependent on the length of rotation trajectory
        elif config_mode == {'file'}:
            raise NotImplementedError
        
    
    def bearing_vectors(self,target):
        """
        Compute bearing vectors from each position point in the trajectory to either a fixed global point,
        or to each point in another (equally sized) trajectory
        params
            target: 1D array of length 3, or 2D array of shape (n,3), where n = self.n
        returns:
            bearing_vectors: 2D array of shape (n,3)
        """
        n = self.n
        if target.shape==(3,) or target.shape==(n,3):
            bearing_vectors = target - self.pos.values
            return bearing_vectors
        else:
            raise ValueError('bearing vector target must have shape (3,) or ({},3), got shape {}'.format(n,target.shape))

       
    

from bspline_core import BSplineCore
class BSpline(TranslationTrajectory):
    """
    BSpline object - contains all of the bspline parameters and methods for evaluating uniform bspline and its derivatives
    """
    def __init__(self, res=100, degree=3, span_time=1, geometric_only=False, control_points=None): #config is a dictionary loaded from yaml
        self.res = res
        self.degree = degree
        self.span_time = float(span_time)
        self.geometric_only = geometric_only
        self.control_points = np.array(control_points).astype(np.float64)
        self.core = BSplineCore(res,degree)
        _dur = span_time*(len(control_points)-degree) # total duration in seconds
        _pos, _vel, _acc = self.evaluate()
        super().__init__(pos=_pos, vel=_vel, acc=_acc, dur=_dur, trajectory_type='bspline')
        
        # control_points_reduced, inds = np.unique(control_points,axis=0, return_index=True) 
   
    def update_config(self,**kwargs):
        """
        update the configuration parameters
        """
        allowed_keys = {'res','degree','span_time','control_points', 'geometric_only'}
        self.__dict__.update((k, v) for k, v in kwargs.items() if k in allowed_keys)
        if 'res' in kwargs or 'degree' in kwargs:
            core = BSplineCore(res,degree)
        

    def evaluate(self):
        """
        compute uniform bspline and its derivatives
        """
        control_points = self.control_points
        core = self.core
        span_time = self.span_time
        geometric_only = self.geometric_only

        k = core.degree
        M = core.get_blending_matrix(k) 
        n = core.res
        l = len(control_points) 
        N = n*(l-k) #total number of points in (each) spline

        pos = np.zeros((N,3)) 
        vel = np.zeros((N,3))
        acc = np.zeros((N,3))
        
        #could just loop to the end to handle any number of derivatives, just do up to acceleration for now
        all_basis = core.basis_vectors
        pos_basis = core.basis_vectors[0,:,:] #nxk
        vel_basis = core.basis_vectors[1,:,:] #nxk
        acc_basis = core.basis_vectors[2,:,:] #nxk
        for i in range(l-k): #todo: avoid this loop
            pts = control_points[i:i+k+1,:] #sliding window of (k+1) control points
            lower = i*n
            upper = (i+1)*n
            pos[lower:upper,:] = np.linalg.multi_dot((pos_basis,M,pts)) #(n x K)*(K x K)*(K x 3)=(n x 3), where K = k+1, n = number of points per span
            vel[lower:upper,:] = np.linalg.multi_dot((vel_basis,M,pts))*(1.0/span_time)
            acc[lower:upper,:] = np.linalg.multi_dot((acc_basis,M,pts))*(1.0/span_time)**2


        #make array of every control point window
        # K = k+1
        # s = l-k
        # from numpy.lib.stride_tricks import sliding_window_view
        # control_point_windows = sliding_window_view(control_points, window_shape=(K,3))
        # #(1 x n x K)*(1 x K x K)*(s x K x 3)=(s x n x 3).reshape(s*n x 3)
        # pos2 = np.squeeze(np.linalg.multi_dot((pos_basis[np.newaxis,:,:],M[np.newaxis,:,:],control_point_windows))).reshape(s*n,3)
        # assert np.allclose(pos,pos2)
        
        return pos, vel, acc

    
    @classmethod
    def config(cls,config):
        config = check_keys(config, 'bspline', context='translation')[0]
        ctrl_pt_config, ctrl_pt_config_mode = check_keys(config.pop('control_points'), 'control_points', context='bspline')
        if ctrl_pt_config_mode == {'points'}:
            points = np.array(ctrl_pt_config['points']).astype(np.float64)
        elif ctrl_pt_config_mode == {'file'}:
            raise NotImplementedError
    
        return cls(**config,control_points=points)
