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

    use: a1 = aligned axis, a2 = grounded axis, a3 = third axis
    """
    remaining = [0,1,2] #axis numbers corresponding to 'xyz'
    try:
        aligned_num = 'xyz'.index(axis)
        remaining.remove(aligned_num)
        if not grounded_axis:
            grounded_num = (aligned_num+1)%3 #next sequential axis number
        elif grounded_axis==axis:
            raise ValueError('axes cannot be the same')
        else:
            grounded_num='xyz'.index(grounded_axis)
        remaining.remove(grounded_num)
        third_num = remaining[0] #whichever the last remaining axis number is
    except ValueError:
        print('axis names must be \'x\',\'y\', or \'z\'')
        raise 

    positive_seq = bool(axis+grounded_axis in ['xy','yz','zx'])
    tol = 1e-6 #tolerance for nearly vertical alignment vectors
    sign = -1.0 if flip else 1.0

    if vec.ndim == 1: #handle the case of a single vector
        a1 = unit(vec)
        x,y,z = a1
        if 1.0-abs(z) < tol:
            a2 = sign*np.array([1.,0.,0.])
        else:
            a2 = sign*np.array([y,-x,0])
        a3 = (1.0 if positive_seq else -1.0)*np.cross(a1,a2)
        R = np.zeros((3,3))
        R[:,aligned_num] = a1
        R[:,grounded_num] = a2
        R[:,third_num] = a3
        return R

    n = len(vec)
    a1 =np.zeros((n,3)).astype(np.float32)
    a2 =np.zeros((n,3)).astype(np.float32)
    a3 =np.zeros((n,3)).astype(np.float32)
    R = np.zeros((n,3,3)) #output array of rotation matrices
    
    a1[:,:] = unit(vec) #first axis
    abs_z = np.abs(a1[:,2]).flatten()
    mask = 1.0-abs_z>tol #False where z is nearly 1 or -1

    #for now, enforce that the first and last entry in 'vec' are not nearly vertical
    #TODO: don't enforce this, find nearest non-vertical vector from both ends and the corresponding grounded axis, use that as the grounded axis
    if not mask[0] or not mask[-1]:
        raise ValueError('rotation_align_axis: first and last alignment vector must not be nearly vertical')

    # for each non-vertical alignment axis [x,y,z]: grounded axis = [y,-x,0] (or [-y,x,0] if flip=True)
    a2[mask,0] =  a1[mask,1]
    a2[mask,1] =  -a1[mask,0]
    a2 = unit(a2) #renormalize
    
    mask = mask.astype(int)
    start = np.where(np.diff(mask)==-1)[0] #indices right before each group of vertical vectors starts
    end = np.where(np.diff(mask)==1)[0]+1 #indices right after each group of vertical vectors ends
    assert len(start)==len(end)

    num_groups = len(start) #number of groups of nearly vertical vectors within 'vec' array
    for i in range(num_groups):
        s = start[i]
        e = end[i]
        ng = e-s+1 #number of vectors in the group, including the two bounding vectors
        a2[s:e+1,:] = unit(linear_interp(a2[s],a2[e],ng))

    a2[:,:] = sign*a2 #negates the 'grounded' axis if flip=True, since there are two possible solutions
    a3[:,:] = (1.0 if positive_seq else -1.0)*np.cross(a1,a2) #third axis

    R[:,:,aligned_num] = a1
    R[:,:,grounded_num] = a2
    R[:,:,third_num] = a3

    # for i in np.arange(n): #debug
    #     print('i: {} a1: {} a2: {} a3: {}'.format(i,a1[i],a2[i],a3[i]))
    return R
    

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
        # block = block_diag(*R_half[0:k-1])  #block diag and stack not necessary, use broadcasting
        # stack = stack_2d(R_half[1:k])              
        # rel = (block.T@stack).reshape(k-1,3,3)
        rel = R_half[0:k-1].swapaxes(1,2)@R_half[1:k]
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
    # angvel = (block_diag(*R)@body_angvel.flatten()).reshape(n,3) #not necessary, use broadcasting
    angvel = (R@(body_angvel.reshape(n,3,1))).reshape(n,3)


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
    """
    #if its already a scipy roation object, return it
    if isinstance(rot,Rotation):
        return rot
    n = len(rot)
    dim = rot.shape
    if dim == (4,) or dim == (n,4):
        out = Rotation.from_quat(rot) #assumes hamilton
    elif dim == (3,3) or dim == (n,3,3):
        out = Rotation.from_matrix(rot)
    elif dim == (3,) or dim == (n,3):
        out = Rotation.from_euler('xyz', rot,degrees=True)
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
    if not q.shape in [(4,), (n,4)]:
        raise ValueError('\'q\' must have shape (4,) or (n,4), got shape {}, n={}'.format(q.shape,n))
    
    q_conj = np.zeros(q.shape)
    q_conj[:,0:3] = -q[:,0:3]
    q_conj[:,3] = q[:,3]
    return q_conj
    
    
def config_transform(config):
    rotation  = np.array(config['rotation']).astype(float)
    rot = as_scipy_rotation(rotation)
    pos = np.array(config['translation']).astype(float)
    return (rot,pos)

def time_derivative(order,data,dt):
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


def random_point_set(n, radius, center):
    """
    generate 'n' random points uniformly distributed within a sphere with 'radius' and 'center'
    """
    c = center
    r_max = radius

    v = np.random.uniform(-1,1,(n,3))
    u = (v.T/np.linalg.norm(v,axis=1)) #random unit vectors
    r = np.cbrt(np.random.uniform(0,r_max**3,n)) #random scales, prevent clustering at center
    points = np.multiply(u,r).T.astype(np.float32) + np.tile(c,(n,1))
    return points

def planar_point_set(radius, center, normal, n=None, grid_spacing=None):
    c = center
    r_max = radius
    if grid_spacing is not None:
        _min = -r_max/2
        _max = r_max/2+grid_spacing #add one grid_spacing to the maximum to include the last point
        X,Y = np.mgrid[_min:_max:grid_spacing, _min:_max:grid_spacing]
        xy = np.vstack((X.ravel(),Y.ravel())).T
        points_xy = np.hstack((xy,np.zeros((xy.shape[0],1)))).astype(np.float32)
        n = len(points_xy)
    elif n is not None:
        v = np.hstack((np.random.uniform(-1,1,(n,2)),np.zeros((n,1)))) #z component is zero
        u = (v.T/np.linalg.norm(v,axis=1)) #random unit vectors on xy plane
        r = np.sqrt(np.random.uniform(0,r_max**2,n)) #random scales, prevent clustering at center
        points_xy = np.multiply(u,r).T.astype(np.float32)
    else:
        raise ValueError('provide either number of points or grid_spacing')
    # print('center: {}\nradius: {}\nnormal: {}\npoints_xy: {}'.format(center.shape,radius.shape,normal.shape,points_xy.shape))
    R = rotation_align_axis(axis='z', vec=normal, grounded_axis='x')
    #print the shape of all of these variables:
    points = (R@points_xy.T).T + np.tile(center,(n,1))
    return points



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

def linear_interp(a,b,n): # TODO change this to (n,*args) so that it can interpolate between multiple vectors
    """
    compute linear interpolation with 'n' steps from vector 'a' to vector 'b', including 'a' and 'b'
    returns: array of shape (n,3)
    """
   
    t = np.linspace(0,1,n).reshape(-1,1)
    return (1-t)*a + t*b

def linear_interp_so3(a,b,n):
    """
    compute linear interpolation with 'n' steps from rotation matrix 'a' to rotation matrix 'b', including 'a' and 'b'
    returns: array of shape (n,3,3)
    """
    R = Rotation.from_matrix([a,b])
    return R.as_matrix()

def get_correspondences(frame_id, index):
    pass
def get_2d_correspondences():
    pass
def get_3d_correspondences():
    pass


def index_from_timestamp(stamps,value):

    """
    returns the index of the closest match to 'value' - used from timestamp matching
    diff = time difference:  value + diff  = closest time stamp
    """
    index = (np.abs(stamps- value)).argmin()
    diff = stamps[index] - value
    return index, diff


