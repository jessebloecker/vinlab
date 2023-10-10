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
    if vec.ndim==1:
        return vec/np.linalg.norm(vec)
    elif vec.ndim==2 and vec.shape[1]==3:
        norms=np.linalg.norm(vec,axis=1)
        return (vec.T/norms).T
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
           expressed in the same frame as 'axis'. If multiple vectors are provided,
           R will be a 3d array with shape (3,3,n)

    use: a1 = axis, a2 = grounded_axis, a3 = last_axis
    """
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

    if vec.ndim==1:
        vec = vec.reshape((1,3))
    n = vec.shape[0]
    
    a1 = unit(vec) #first axis
    a2 = (-1.0 if flip else 1.0)*np.vstack((a1[:,1],-a1[:,0],np.zeros(n))).T #second axis ("grounded axis")

    positive_seq=['xy','yz','zx'] #ensure the result of the cross product will be a right-handed frame
    sign = 1.0 if axis+grounded_axis in positive_seq else -1.0
    a3 = sign*np.cross(a1,a2)

    positive_seq = bool(axis+grounded_axis in ['xy','yz','zx'])
    a3 = (1.0 if positive_seq else -1.0)*np.cross(a1,a2)

    # print(axis_num,grounded_num,last_num)
    R = np.zeros((3,3,n))
    R[:,axis_num,:] = a1.T
    R[:,grounded_num,:] = a2.T
    R[:,last_num,:] = a3.T

    return np.squeeze(R)
    

def rot_seq_to_angvel(seq,dt,frame='moving'):
    """
    compute angular velocity from a sequence of global rotation matrices at uniform time intervals
    params:
        seq: 3d array of shape (n,3,3) where seq[i,:,:] is the rotation matrix at time i
        dt: float, time step between each rotation matrix
    returns:
        angvel: 2d array of shape (n,3) where angvel[i,:] is the angular velocity at time i expressed in the moving frame
    """
    # n=len(seq)
    # seq_block = block_diag(*seq) #3n x 3n
    # forward_seq = np.copy(seq)
    # backward_seq = np.copy(seq)
    # forward_seq[:-1,:,:] = seq[1:,:,:] # remove first entry, duplicate last entry
    # backward_seq[1:,:,:] = seq[:-1,:,:] # remove last entry, duplicate first entry
   
    # forward_2d = forward_seq.reshape(3*n,3) #3n x 3
    # backward_2d = backward_seq.reshape(3*n,3) #3n x 3
    
    # forward_rel_seq = (seq_block.T@forward_2d.reshape(n,3,3)) #3n x 3n remove first duplicate last
    # backward_rel_seq = (seq_block.T@backward_2d.reshape(n,3,3)) #3n x 3n remove last duplicate first
    
    # rotvecs1 = Rotation.from_matrix(forward_rel_seq).as_rotvec()
    # rotvecs2 = Rotation.from_matrix(backward_rel_seq).as_rotvec()
    
    # angles1 = np.linalg.norm(rotvecs1,axis=1)
    # angles2 = np.linalg.norm(rotvecs2, axis=1)

    # angvels = (1/(2*dt))*(angles1+angles2) #scalar
    # axes = unit((unit(rotvecs1)+unit(rotvecs2))/2)

    # angvel_vecs = (axes.T*angvels).T
   
    # return angvel_vecs


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


def centroid(arr):
    """
    compute the centroid of a set of position points
    params:
        arr: 2d array of position points with shape (n,3)
    returns:
        centroid position as 1d array of shape (3,)
    """
    return np.mean(arr,axis=0)