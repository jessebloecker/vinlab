#!/usr/bin/env python
import numpy as np

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
    

def find_closest(v,values, max_diff):
    """
    v:  the value to be matched
    values: 1-d array of values 

    return the index of the value closest to 'v'
    """
    closest = np.abs(values-v).argmin()
    if np.abs(values[closest]-v) <= max_diff:
        return closest
    else:
        return -1

def value_match(a,b,max_diff):
    """
    Given two 1D arrays, find the indices of the values in 'a' that are closest to the values in 'b', and vice versa.
    params:
        a (1D array): first array
        b (1D array): second array
        max_diff (float): maximum difference between two values for them to be considered a correspondence
    returns:
        cors_a (1D array): same size as 'a', containing the indices of the corresponding value in 'b'
        cors_b (1D array): same size as 'b', containing the indices of the corresponding value in 'a'
    """
    # a = a.reshape(-1,1)
    # b = b.reshape(-1,1)
    range_a = np.arange(len(a))
    range_b = np.arange(len(b))

    ab = np.apply_along_axis(find_closest, axis=1, arr=a.reshape(-1,1), values=b, max_diff=max_diff)
    ba = np.apply_along_axis(find_closest, axis=1, arr=b.reshape(-1,1), values=a, max_diff=max_diff)
    cors_ab = np.vstack((range_a, ab)).T
    cors_ba = np.vstack((range_b, ba)).T

    #intersect the two correspondence sets (reject correspondences that don't occur in both directions)
    cors_ab_set = set(map(tuple, cors_ab))
    cors_ba_set = set(map(tuple, np.flip(cors_ba))) #flip cors_ba so that the tuples to be compared are always in order (a,b)

    # cors = np.array([x for x in cors_ab_set & cors_ba_set])
    cors = np.array(list(cors_ab_set & cors_ba_set))


    cors_a = -1*np.ones(len(a)).astype(int)
    cors_b = -1*np.ones(len(b)).astype(int)
    if cors.size>0:
        cors_sorted = cors[cors[:,0].argsort()] if cors.size>0 else cors  # put it back in order (sort by 'a' index)
        # cors = cors[cors[:,1].argsort()] # (sort by 'b' index)

        cors_a[cors[:,0]] = cors[:,1] #value is -1 except where theres a valid correspondence, in which case the value is the corresponding index in b
        cors_b[cors[:,1]] = cors[:,0] #value is -1 except where theres a valid correspondence, in which case the value is the corresponding index in a

        assert len(cors) == len(cors_a[cors_a!=-1]) == len(cors_b[cors_b!=-1]), 'number of correspondences found does not match'
        print('cors_a: {}'.format(cors_a))
        print('cors_b: {}'.format(cors_b))

        value_correspondences = np.vstack((a[cors_a>=0],b[cors_b>=0])).T
        # print('ab: {}'.format(ab))
        # print('ba: {}'.format(ba))
        # print('cors_ab:\n {}'.format(cors_ab))
        # print('cors_ba: \n{}'.format(cors_ba))
        print('{} correspondences: \n{}'.format(len(cors),cors_sorted))
        print('corresponding values')
        print(value_correspondences)
    else:
        print('no correspondences found for array \'a\' of size {} and array \'b\' of size {}'.format(len(a),len(b)))


