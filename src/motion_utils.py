#!/usr/bin/env python
import numpy as np
from scipy.spatial.transform import Rotation


def angvel_from_rotations(rot,dt):
    """
    compute angular velocity from a sequence of global rotation matrices at uniform time intervals
    params:
        seq: 3d array of shape (n,3,3) where seq[i,:,:] is the rotation matrix at time i
        dt: float, time step between each rotation matrix
    returns:
        (angvel, body_angvel): 2d array of shape (n,3) where angvel[i,:] is the angular velocity at time i expressed in the global frame and body frame
    
    Example
     n = 10  --->  k = 5
     R = [0,1,2,3,4,5,6,7,8,9]
    
       odd        1 | 3 | 5 | 7 | (9)       relative rotation at each even index comes from
                    |   |   |   |          the product of the two neighboring (odd) rotations, and vice versa   
       even   (0) | 2 | 4 | 6 | 8           endpoints are handled separately at the end
                  |   |   |   |             
                  1 | 3 | 5 | 7 | (9)
    
    """
    R = rot
    n = len(R)
    R_rel = np.zeros((n,3,3))
    
    n_is_odd = bool(n%2)
    if n_is_odd: #reduce n by 1, then correct for endpoints later
        n = n-1  

    k = int(n/2)   #half the number of rotations 
    R_e = R[0::2]  #rotations at even indices - shape (k,3,3))
    R_o = R[1::2]  #rotations at odd indices - shape (k,3,3))

    i = 0
    for R_half in (R_e,R_o): 
        rel = R_half[0:k-1].swapaxes(1,2)@R_half[1:k]
        R_rel[1:n-1][i::2] = rel # 'interlace' the relative rotations back into one (n,3,3) array
        i+=1

    # now handle the endpoints
    if n_is_odd:
        n+=1 # the previously last rotation is now the second to last rotation, compute relative rotation at n-2 exactly as above
        R_rel[n-2] = R[n-3].T@R[n-1]
    R_rel[0] = R[0].T@R[1] #forward difference
    R_rel[n-1] = R[n-2].T@R[n-1] #backward difference

    rotvec = Rotation.from_matrix(R_rel).as_rotvec()
    angvel = np.zeros((n,3))
    body_angvel = np.zeros((n,3))

    # for i in range(0,len(rotvec)):
    #     u = unit(rotvec[i])
    #     u_prev = unit(rotvec[i-1])
    #     print('{}: axis {}, diff {} angle {}'.format(i,u,np.linalg.norm(u-u_prev),np.rad2deg(np.linalg.norm(rotvec[i]))))

    body_angvel[0] = (1/dt)*rotvec[0]
    body_angvel[1:n-1,:] = (1/(2*dt))*rotvec[1:n-1,:]
    body_angvel[n-1] = (1/dt)*rotvec[n-1]
    angvel = (R@(body_angvel.reshape(n,3,1))).reshape(n,3)

    return angvel, body_angvel


def time_derivative(order,data,dt):
    """
    Wrapper around np.gradient: compute any order time derivative of a series of points, given dt
    params:
        data: nx3 array of data
        order: order of derivative 
        dt: time step
    """
    out = data
    for i in range(order):
        out = np.gradient(out,axis=0)/dt
    return out


def init_trajectory_timestamps(n, t=None, dur=None):
    """
    initialize timestamps array from either given time stamps or duration
    params:
        n: number of data points
        t: array of time stamps
        dur: duration of trajectory
    returns:
        tuple (_t,_dur), where
            _t = timestamps array of shape (n,)
            _dur = duration (difference between first and last time stamps)
    """
    if t is not None:
        if len(t)==n:
            _t = t
            _dur = float(_t[-1]-_t[0]) #ignore provided 'dur'
        else:
            raise ValueError('number of time stamps ({}) does not match number of data points ({})'.format(len(t),n))
    elif dur is not None:
        _t = np.linspace(0,dur,n)
        _dur = float(dur)
    else:
        raise ValueError('must provide either a duration or an array of time stamps')
    return _t, _dur




