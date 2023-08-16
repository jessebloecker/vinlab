#!/usr/bin/env python 
import numpy as np


class BSplineCore():
    """
    Evaluate and store the basis vector (and its derivatives), and the corresponding blending 
    matrix, given bspline order and array of time values.

    params: 
        t: time values (1d or 0d array)
        k: order of the bspline, default = 3 (cubic)
    """
    def __init__(self,t,k):
        self.t = t
        self.order = k
        self.basis_vectors = self.eval_basis_vectors(t,k)
        self.blending_matrix = self.get_blending_matrix(k)

    def eval_basis_vectors(self,t,k,d=None):
        """
        Compute a stack of arrays: each 'layer' (along 1st axis) is an array corresponding to a derivative
        of the original basis vector. 

        (layer 0 = basis, layer 1 = first derivative basis, etc.)
    
        In a given layer, each row is the current basis vector evaluated at a different time value. 
        params:
            t: time values (1d or 0d array) of length n
            k: order of the bspline, default = 3 (cubic)
            d: include all derivatives up to this order:
               if None (default), include all derivatives, 
               if 0, include only the original spline)
        returns: b: 3D array (d x n x k)
        """
        n = len(t)
        max_k = 8

        if d is None or d < 0:
            d = k
        elif d > k:
            raise ValueError('d ({}) must be less than or equal to order k ({})'.format(d,k))
        if k > max_k:
            raise ValueError('order k ({}) must be less than or equal to {}'.format(k,max_k))
        
        B = np.array([[1*t**0, t, t**2, t**3, t**4, t**5, t**6, t**7],                       #basis vector
                     [0*t, 1*t**0, 2*t, 3*t**2, 4*t**3, 5*t**4, 6*t**5, 7*t**6],             #first derivative
                     [0*t, 0*t, 2*t**0, 6*t, 12*t**2, 20*t**3, 30*t**4, 42*t**5],            #second derivative
                     [0*t, 0*t, 0*t, 6*t**0, 24*t, 60*t**2, 120*t**3, 210*t**4],             # .
                     [0*t, 0*t, 0*t, 0*t, 24*t**0, 120*t, 360*t**2, 840*t**3],               # .
                     [0*t, 0*t, 0*t, 0*t, 0*t, 120*t**0, 720*t, 2520*t**2],                  # .
                     [0*t, 0*t, 0*t, 0*t, 0*t, 0*t, 720*t**0, 5040*t],                       # .
                     [0*t, 0*t, 0*t, 0*t, 0*t, 0*t, 0*t, 5040*t**0]])                        # seventh derivative
        B = np.swapaxes(B,1,2) #make dimensions (d x n x k) as described
        b = B[0:d+1,:,0:k+1] #select the first d+1 layers and first order+1 columns in each layer
        return b
    
    def get_blending_matrix(self, order):
        """
        return list of blending matrices - one for each order of bspline
        """
        if order==0:
            M = np.array([1.0])
        elif order==1:
            M = (1/6.0)*np.array([[1, 0],
                                [-1, 1]],dtype=np.float64)
        elif order==2:
            M = (1/2.0)*np.array([[1,1,0],
                                [-2,2,0],
                                [1,-2,1]],dtype=np.float64)
        elif order==3:   
            M = (1/6.0)*np.array([[ 1,  4, 1, 0],
                                [-3,  0, 3, 0],
                                [ 3, -6, 3, 0],
                                [-1,  3,-3, 1]],dtype=np.float64)
        elif order==4:        
            M = (1/24.0)*np.array([[1, 11, 11, 1, 0],
                                    [-4, -12, 12, 4, 0],
                                    [ 6, -6, -6, 6, 0],
                                    [-4, 12, -12, 4, 0],
                                    [ 1, -4, 6, -4, 1]],dtype=np.float64)
        else:
            raise ValueError('order must be 0,1,2,3, or 4')
        return M
    
if __name__ == '__main__':
    s = BSplineCore()
    bsplines = s.get_bsplines(3,s.control_points,s.resolution,0,1.0)
    # vel = s.bspline(3,1,s.control_points,s.resolution,0,1.0)
    # acc = s.bspline(3,2,s.control_points,s.resolution,0,1.0)
    # jerk = s.bspline(3,3,s.control_points,s.resolution,0,1.0)

"""
bspline_core.py: generate all of the splines given control_points
bspline_ros.py: publish everything according to some display controller
                        to turn things on and off
bspline_controller.py: clickable buttons to turn things on and off and edit the spline
bspline_gui.py front end for bspline_controller.py
bspline_evaluator.py: compute_bspline the splines, find max and min of each derivative
                                total length, total time, radius of curvature.
imu_simulator.py: simulate imu data given the splines
"""

# gui->controller->core->eval->ros


