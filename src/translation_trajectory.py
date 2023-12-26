#!/usr/bin/env python 

import numpy as np
import motion_utils as utils

class TranslationTrajectory():
    """"
    trajectory containing position, velocity, and acceleration
    """
    def __init__(self, pos, t=None, dur=None, vel=None, acc=None):

        n = len(pos)
        _t, _dur = utils.init_trajectory_timestamps(n,t,dur)
        
        dt = _dur/(n-1)
        numerical_vel = self.time_derivative(1,pos,dt)
        numerical_acc = self.time_derivative(1,numerical_vel,dt)

        _vel = numerical_vel if (vel is None) else vel
        _acc = numerical_acc if (acc is None) else acc

        self.pos = pos
        self.vel = _vel
        self.acc = _acc
        self.numerical_vel = numerical_vel
        self.numerical_acc = numerical_acc
        self.n = n
        self.dur = _dur
        self.t = _t
        self.dt = dt
        self.rate = 1./dt

    @classmethod
    def time_derivative(cls,order,data,dt):
        """
        Compute any order time derivative of a series of points, given dt
        params:
            data: nx3 array of data
            order: order of derivative 
            dt: time step
        """
        out = data
        for i in range(order):
            out = np.gradient(out,axis=0)/dt
        return out
    



from bspline_core import BSplineCore
class BSpline(TranslationTrajectory):
    """
    BSpline object - contains all of the bspline parameters and methods for evaluating uniform bspline and its derivatives
    """
    def __init__(self, res=100, order=3, span_time=1, control_pts=None): #config is a dictionary loaded from yaml
        # self.load_config(config_path)/

        self.res = res
        self.order = order
        self.span_time = float(span_time)
        self.control_pts = np.array(control_pts).astype(np.float64)
        self.core = BSplineCore(res,order)
        _dur = span_time*(len(control_pts)-order) # total duration in seconds
        _pos, _vel, _acc = self.evaluate()
        super().__init__(pos=_pos, vel=_vel, acc=_acc, dur=_dur)
        
        # control_pts_reduced, inds = np.unique(control_pts,axis=0, return_index=True) 
   

    def update_config(self,**kwargs):
        """
        update the configuration parameters
        """
        allowed_keys = {'res','order','span_time','control_pts'}
        self.__dict__.update((k, v) for k, v in kwargs.items() if k in allowed_keys)
        if 'res' in kwargs or 'order' in kwargs:
            core = BSplineCore(res,order)
        

    def evaluate(self):
        """
        compute uniform bspline and its derivatives
        """
        control_pts = self.control_pts
        core = self.core
        span_time = self.span_time

        k = core.order
        M = core.get_blending_matrix(k) 
        n = core.res
        l = len(control_pts) 
        N = n*(l-k) #total number of points in (each) spline

        
        #could just loop to the end to handle any number of derivatives. prob gonna have to do this
        all_basis = core.basis_vectors
        pos_basis = core.basis_vectors[0,:,:] #nxk
        vel_basis = core.basis_vectors[1,:,:] #nxk
        acc_basis = core.basis_vectors[2,:,:] #nxk

        pos = np.zeros((N,3)) 
        vel = np.zeros((N,3))
        acc = np.zeros((N,3))

        for i in range(l-k): #could probably make this whole thing a matrix operation somehow (make giant diagonal matrix...i've done this before))
            #also, after the first time, you only have to update sections of the bspline each time, not recompute the whole thing
            pts = control_pts[i:i+k+1,:] #sliding window of (k+1) control points
            lower = i*n
            upper = (i+1)*n
            pos[lower:upper,:] = np.linalg.multi_dot((pos_basis,M,pts)) #(n x k1)*(k1 x k1)*(k1 x 3)=(n x 3), where k1 = order+1
            vel[lower:upper,:] = np.linalg.multi_dot((vel_basis,M,pts))*(1.0/span_time)
            acc[lower:upper,:] = np.linalg.multi_dot((acc_basis,M,pts))*(1.0/span_time)**2

        return pos, vel, acc

    

 