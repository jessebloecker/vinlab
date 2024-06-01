#!/usr/bin/env python
import numpy as np
from array_utils import unit
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation

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

def in_frame(rot,vec):
    """
    multiply each vector in 'vec' by the inverse of corresponding rotation in 'rot'
    params:
        vec: array (n,3)
        rot: array (n,3,3) 
    returns:
        vec_rotated: array (n,3) where vec_rotated[i,:] = rot[i,:,:].T@vec[i,:]
    """
    n = len(rot)
    if vec.ndim == 1:
        vec = vec[np.newaxis,:].repeat(len(rot),axis=0)
    
    
    R = block_diag(*rot)
    vec_rotated = (R.T@vec.flatten()).reshape(vec.shape)

    vec_rotated2 = (rot.swapaxes(1,2)@vec.reshape(n,3,1)).reshape(n,3) #just use this line where needed, don't need this function
    assert np.allclose(vec_rotated,vec_rotated2)


    return vec_rotated

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



def random_point_set(center, radius, num):
    """
    generate 'n' random points uniformly distributed within a sphere with 'radius' and 'center'
    """
    c = center
    n = num
    r_max = radius

    v = np.random.uniform(-1,1,(n,3))
    u = (v.T/np.linalg.norm(v,axis=1)) #random unit vectors
    r = np.cbrt(np.random.uniform(0,r_max**3,n)) #random scales, prevent clustering at center
    points = np.multiply(u,r).T.astype(np.float32) + np.tile(c,(n,1))
    return points


def planar_point_set(center, normal, radius, num=None, grid_spacing=None):
    c = center
    n = num
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
        raise ValueError('planar_point_set: provide either number of points or grid_spacing')
    # print('center: {}\nradius: {}\nnormal: {}\npoints_xy: {}'.format(center.shape,radius.shape,normal.shape,points_xy.shape))
    R = rotation_align_axis(axis='z', vec=normal, grounded_axis='x')
    #print the shape of all of these variables:
    points = (R@points_xy.T).T + np.tile(center,(n,1))
    return points

def linear_interp(a,b,n): 
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
    pass
