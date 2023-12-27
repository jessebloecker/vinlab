#!/usr/bin/env python
import numpy as np
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation

def unit(vec):
    """
    normalize vector to unit length
    params:
        vec: 1d array or 2d array of shape (n,3)
    returns:
        unit vector or nx3 array of unit vectors
    """
    if not np.any(vec):
        raise ValueError('zero vector cannot be normalized') 
    if vec.ndim == 1:
        return vec/np.linalg.norm(vec)
    elif vec.ndim == 2 and vec.shape[1]==3:
        norms = np.linalg.norm(vec,axis=1)
        out = (vec.T/norms).T  #causes RuntimeWarning: invalid value encountered in true_divide
        return np.nan_to_num(out) #replace NaNs with zeros TODO: find a better way to handle this

    else:
        raise ValueError('\'vec\' must be a 1d array, or a 2d array with shape (n,3)')
    
def rotation_align_axis(axis,vec,grounded_axis=None,flip=False):
    """
    compute the rotation(s) that aligns given axis with the given vector(s), 
    and keeps one other axis parallel to the original xy-plane (ground plane)

    params:
        axis: string, 'x','y', or 'z' axis to align with vec
        vec: 1d array of length 3 (vector to align axis with) or 2d array of these vectors with shape (n,3)
        grounded: string, 'x','y', or 'z' axis to remain parallel to the original xy-plane.
                  cannot be the same as 'axis' param.
                  if not provided, it will be the next sequential axis after 'axis' (x->y, y->z, z->x).
        flip: bool, if True, the grounded axis will be negated, giving the other available solution
    returns:
        R: rotation matrix where columns are the unit vectors of aligned frame 
           expressed in the same frame as provided vectors. If multiple vectors are provided,
           R will be a 3d array with shape (3,3,n)

    use: a1 = axis, a2 = grounded_axis, a3 = last_axis
    """
    n = vec.shape[0]
    a1 =np.zeros((n,3))
    a2 =np.zeros((n,3))
    a3 =np.zeros((n,3))
    R = np.zeros((n,3,3))

    remaining = [0,1,2] #axis numbers corresponding to 'xyz'
    try:
        axis_num = 'xyz'.index(axis)
        remaining.remove(axis_num)
        if not grounded_axis:
            grounded_num = (axis_num+1)%3 #next sequential axis number
        elif grounded_axis==axis:
            raise ValueError('axes cannot be the same')
        else:
            grounded_num='xyz'.index(grounded_axis)
        remaining.remove(grounded_num)
        last_num = remaining[0] #whichever the last remaining axis number is
    except ValueError:
        print('axis names must be \'x\',\'y\', or \'z\'')
        raise 

    a1[:,:] = unit(vec) #first axis
    
    a2[:,0] =  a1[:,1] 
    a2[:,1] =  -a1[:,0]
    a2[:,:] = (-1.0 if flip else 1.0)*unit(a2)  #second axis ("grounded_axis")

    positive_seq = bool(axis+grounded_axis in ['xy','yz','zx'])
    a3[:,:] = (1.0 if positive_seq else -1.0)*np.cross(a1,a2) #third axis

    R[:,:,axis_num] = a1
    R[:,:,grounded_num] = a2
    R[:,:,last_num] = a3

    # for i in np.arange(n): #debug
    #     print('i: {} a1: {} a2: {} a3: {}'.format(i,a1[i],a2[i],a3[i]))
    return np.squeeze(R)
    

def angvel_from_rotations(rot,dt):
    """
    compute angular velocity from a sequence of global rotation matrices at uniform time intervals
    params:
        seq: 3d array of shape (n,3,3) where seq[i,:,:] is the rotation matrix at time i
        dt: float, time step between each rotation matrix
    returns:
        angvel: 2d array of shape (n,3) where angvel[i,:] is the angular velocity at time i expressed in the moving frame
    
    Example
     n = 10  --->  k = 5
     R = [0,1,2,3,4,5,6,7,8,9]
    
       odd        1 | 3 | 5 | 7 | (9)       relative rotation at each even index comes from
                    |   |   |   |          the product of the two neighboring (odd) rotations, and vice versa   
       even   (0) | 2 | 4 | 6 | 8           endpoints are handled separately at the end
                  |   |   |   |             
                  1 | 3 | 5 | 7 | (9)


    relative rotations computed by creating a block diagonal matrix 
    and a corresponding stacked matrix, once for the odd indices, and once for the even indices:

    -----------------------------------------------------------------------------       
                       T
    |R1               |     |   R3   |       |  rel2     |    
    |   R3            |     |   R5   |       |  rel4     |
    |     R5          |  *  |   R7   |    =  |  rel6     | 
    |       ...       |     |  ...   |       |   ...     | 
    |           R(n-3)|     |  R(n-1)|       |  rel(n-2) | 

    block(R_o)           *  stack(R_o)    =    rel_e        <----rel_e = relative rotations at even indices
      from 0 to k-1         from 1 to k      shape(3*k,3)
    
    -----------------------------------------------------------------------------
    
    
                       T
    |R0               |     |   R2   |       |  rel1     |    
    |   R2            |     |   R4   |       |  rel3     |
    |     R4          |  *  |   R6   |    =  |  rel5     | 
    |       ...       |     |  ...   |       | ...       | 
    |           R(n-4)|     |  R(n-2)|       |  rel(n-3) | 

    block(R_e)           *    stack(R_e)  =    rel_o        <----rel_o = relative rotations at odd indices
      from 0 to k-1         from 1 to k       shape(3*k,3)
    
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
        block = block_diag(*R_half[0:k-1])
        stack = stack_2d(R_half[1:k])
        rel = (block.T@stack).reshape(k-1,3,3)
        R_rel[1:n-1][i::2] = rel # 'interlace' the relative rotations back into one (n,3,3) array
        i+=1

    # now handle the endpoints
    if n_is_odd:
        n+=1
        # the previously last rotation is now the second to last rotation
        # compute relative rotation at n-2 exactly as above
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
    angvel = (block_diag(*R)@body_angvel.flatten()).reshape(n,3)

    return angvel, body_angvel

def stack_2d(arr):
    """
    arr: 3d array(n,m,p)
    returns: 2d array(n*m,p)
    """
    if not len(arr.shape)==3:
        raise ValueError('array must be a 3d, got shape {}'.format(arr.shape))
    n, m, p = arr.shape
    return arr.reshape(n*m,p)


def repeat_3d(arr,n):
    """
    stack 'n' copies of  'arr' into a 3d array (stack along first axis)
    params:
        arr: 2d array
        n: int, number of copies to stack
    returns:
        3d array of shape (n, arr.shape[0],arr.shape[1])
    """
    try:
        return arr[np.newaxis,:,:].repeat(n,axis=0)
    except IndexError:
        print('\'arr\' must be a 2d array')
        raise

def in_frame(rot,vec):
    """
    multiply each vector in 'vec' by the inverse of corresponding rotation in 'rot'
    params:
        vec: array (n,3)
        rot: array (n,3,3) 
    returns:
        vec_rotated: array (n,3) where vec_rotated[i,:] = rot[i,:,:].T@vec[i,:]
    """
    if vec.ndim == 1:
        vec = vec[np.newaxis,:].repeat(len(rot),axis=0)
    
    
    R = block_diag(*rot)
    vec_rotated = (R.T@vec.flatten()).reshape(vec.shape)
    return vec_rotated

def skew(vec):
    """
    compute skew symmetric matrix from vector
    params:
        vec: array of length 3
    returns:
        skew symmetric matrix of shape (3,3)
    """

    return np.array([[0,-vec[2],vec[1]],
                     [vec[2],0,-vec[0]],
                     [-vec[1],vec[0],0]])


def as_scipy_rotation(rot):
    """
    return scipy Rotation object based on dimensions of given the input
    return rotation object

     if given: 4x1 vector, assume quaternion [x,y,z,w]
            3x3 matrix, assume rotation matrix
            3x1 vector, assume roll-pitch-yaw = (x,y,z) 
                                with convention x --> y --> z in fixed frame
                                (equivalent to z --> y --> x in body frame)
            1x1 scalar, assume yaw in radians
    """
    n = len(rot)
    dim = rot.shape
    if dim == (4,) or dim == (n,4):
        out = Rotation.from_quat(rot) #assumes hamilton
    elif dim == (3,3) or dim == (n,3,3):
        out = Rotation.from_matrix(rot)
    elif dim == (3,) or dim == (n,3):
        out = Rotation.from_euler('xyz', rot)
    else:
        raise ValueError('Invalid rotation input with shape {}'.format(dim))
    return out

def q_conjugate(q):
    """
    Compute the conjugate of quaternion(s) in [x,y,z,w] format.
    Equivalent to switching between Hamilton and JPL conventions
    params:
        q: (4,) or (4,n) array
    returns:
        q_conj = [-x,-y,-z,w]
    """
    n = len(q)
    if not q.shape in [(4,), (4,n)]:
        raise ValueError('\'q\' must have shape (4,) or (4,n), got shape {}, n={}'.format(q.shape,n))
    
    q_conj = np.zeros(q.shape)
    q_conj[0:3] = -q[0:3]
    q_conj[3] = q[3]
    return q_conj
    
    
def config_transform(config):
    rotation  = np.array(config['rotation']).astype(np.float32)
    R = as_scipy_rotation(rotation).as_matrix()
    pos = np.array(config['translation']).astype(np.float32)
    return (R,pos)


def attribute_print(obj,indent=1,object_name=None):
    if object_name is None:
        object_name = obj.__class__.__name__
    PURPLE = "\033[0;35m"
    YELLOW = "\033[1;33m"
    BOLD = "\033[1m"
    END = "\033[0m"
    space='    '
    print((indent-1)*space+YELLOW+str(object_name)+END)
    for k, v in obj.__dict__.items():
        if v.__class__.__module__=='numpy':
            print(indent*space+BOLD+'{}: '.format(k)+END+
                    PURPLE+'{} '.format(v.shape)+END+'{}'.format(v.dtype))
        elif isinstance(v,list):
            print((indent)*space+YELLOW+str(k.title())+END)
            for i in range(len(v)):
                attribute_print(v[i],indent+2,object_name='_')
        elif not v.__class__.__module__=='builtins':
            attribute_print(v,indent+1)
        else:
            print(indent*space+BOLD+'{}'.format(k)+END+': {}'.format(v))

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
