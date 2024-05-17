#!/usr/bin/env python 
import numpy as np
from math import comb

class BSplineCore():
    """
    Given bspline order and resolution, evaluate and store the basis vector and its
    derivatives at time values from 0 to 1, and the store the corresponding blending 
    matrix

    Attributes:
        res (int): resolution - number of points per span of each bspline
        order (int): bspline order (1=linear, 2=quadratic, 3=cubic, etc.)
        basis_vectors (ndarray): (see function)
        blending_matrix (ndarray): (see function)
    """
    def __init__(self,res,order):
        self.res = res
        self.order = order
        self.basis_vectors = self.eval_basis_vectors(res,order)
        self.blending_matrix = self.get_blending_matrix(order)

    def eval_basis_vectors(self,n,k,d=None):
        """
        Return a 3d array: each 'layer' (along last axis) is 2d array 
        corresponding to a derivative of the original basis vector. 
        (layer 0 = basis, layer 1 = first derivative basis, etc.)
        In a given layer, each row corresponds to current basis vector 
        evaluated at a different time value from 0 to 1.
        params:
            n (int): resolution - number of points per span of each bspline
            k (int): bspline order (1=linear, 2=quadratic, 3=cubic, etc.)
            d (int): include all derivatives up to this order.
                     if None (default), include all derivatives.
                     if 0, include only the original spline.
        returns: 
            3D array (d x n x k)
        """
        max_k = 8

        if d is None or d < 0:
            d = k
        elif d > k:
            raise ValueError('d ({}) must be less than or equal to order k ({})'.format(d,k))
        if k > max_k:
            raise ValueError('order k ({}) must be less than or equal to {}'.format(k,max_k))
        
        t = np.linspace(0,1,n,endpoint=False) #arbitrary time unit, to be scaled later)

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
        params:
            order(int)
        returns:
            ndarray - blending matrix corresponding 'order'
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
            M = (1/24.0)*np.array([[  1,  11,  11,   1,   0],
                                   [ -4, -12,  12,   4,   0],
                                   [  6,  -6,  -6,   6,   0],
                                   [ -4,  12, -12,   4,   0],
                                   [  1,  -4,   6,  -4,   1]],dtype=np.float64)
        elif order==5:
            M = (1/120.0)*np.array([[  1,  26,  66,  26,   1,  0],
                                    [ -5, -50,   0,  50,   5,  0],
                                    [ 10,  20, -60,  20,  10,  0],
                                    [-10,  20,   0, -20,  10,  0],
                                    [  5, -20,  30, -20,   5,  0],
                                    [ -1,   5, -10,  10,  -5,  1]],dtype=np.float64)
            
        elif order==6:
            M = (1/720.0)*np.array([[   1,   57,  302,  302,   57,   1,  0],
                                    [  -6, -150, -240,  240,  150,   6,  0],
                                    [  15,  135, -150, -150,  135,  15,  0],
                                    [ -20,  -20,  160, -160,   20,  20,  0],
                                    [  15,  -45,   30,   30,  -45,  15,  0],
                                    [  -6,   30,  -60,   60,  -30,   6,  0],
                                    [   1,   -6,   15,  -20,   15,  -6,  1]],dtype=np.float64)

        elif order==7:
            M = (1/5040.0)*np.array([[   1,   120,  1191,  2416,  1191,  120,   1,  0],
                                     [  -7,  -392, -1715,     0,  1715,  392,   7,  0],
                                     [  21,   504,   315, -1680,   315,  504,  21,  0],
                                     [ -35,  -280,   665,     0,  -665,  280,  35,  0],
                                     [  35,     0,  -315,   560,  -315,    0,  35,  0],
                                     [ -21,    84,  -105,     0,   105,  -84,  21,  0],
                                     [   7,   -42,   105,  -140,   105,  -42,   7,  0],
                                     [  -1,     7,   -21,    35,   -35,   21,  -7,  1]],dtype=np.float64)

        else:
            raise ValueError('bspline order \'{}\' is invalid or not supported - must an integer between 2 and 7 inclusive.'.format(order))
        return M
    
    @classmethod
    def print_blending_matrix(cls,order):
        """
        print out the blending matrix 'M' for the given bspline order
        params:
            order(int)
        """
        k = order
        M = np.zeros((k+1,k+1)).astype(int)
        for i in range(k+1):
            for j in range(k+1):
                term = 0.0
                for s in range(j,k+1):
                    term += (-1)**(s-j)*comb(k+1,s-j)*(k-s)**(k-i)
                M[i,j] = comb(k,k-i)*term
        print('blending matrix for order {} bspline: \n M = (1/({}!)) * \n{}\n'.format(k,k,M))
    

if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    BSplineCore.print_blending_matrix(2)
    BSplineCore.print_blending_matrix(3)
    BSplineCore.print_blending_matrix(4)
    BSplineCore.print_blending_matrix(5)
    BSplineCore.print_blending_matrix(6)
    BSplineCore.print_blending_matrix(7)
