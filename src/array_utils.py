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
        raise ValueError('\'q\' must have shape (4,) or (n,4), got shape {}'.format(q.shape))
    
    q_conj = np.zeros(q.shape)
    q_conj[:,0:3] = -q[:,0:3]
    q_conj[:,3] = q[:,3]
    return q_conj
    

def find_closest(v,values, tol):
    """
    v:  the value to be matched
    values: 1-d array of values 

    return the index of the value closest to 'v'
    """
    closest = np.abs(values-v).argmin()
    if np.abs(values[closest]-v) <= tol:
        return closest
    else:
        return -1

class ValueMatcher():
    def __init__(self, a,b, tol=0.0):

        cors_a, cors_b = self.value_match(a,b,tol)
        mask_a, mask_b = cors_a>=0, cors_b>=0
        indices_a, indices_b = cors_a[mask_a], cors_b[mask_b]
        assert(len(indices_a)==len(a[mask_a])), 'number of correspondance indices does not match number of correspondences for array a'
        assert(len(indices_b)==len(b[mask_b])), 'number of correspondance indices does not match number of correspondences for array b'
        values_a, values_b = b[indices_a], a[mask_a]
        self.length_a, self.length_b = len(a), len(b)
        self.cors = cors_a, cors_b
        self.masks = mask_a, mask_b
        self.index_cors = indices_a, indices_b
        self.values = values_a, values_b
        # self.cors could be constructed from self.masks and self.index_cors, but useful to have it as a separate variable

    def align(self, list_a,list_b):
        #check tat every array in list_a has the same length as as a, and that every array in list_b has the same length as b
        for i in [0,1]:
            l = list_a if i==0 else list_b
            for arr in l:
                if not len(arr) == (self.length_a,self.length_b)[i]:
                    raise ValueError('arrays in list must have the same length as the corresponding array in the ValueMatcher object')

        for i in range(len(list_a)):
            list_a[i] = list_a[i].reshape(-1,1) if list_a[i].ndim==1 else list_a[i]
        for i in range(len(list_b)):
            list_b[i] = list_b[i].reshape(-1,1) if list_b[i].ndim==1 else list_b[i]

        mask_a, mask_b = self.masks
        stack_a = np.hstack([arr[mask_a] for arr in list_a])
        stack_b = np.hstack([arr[mask_b] for arr in list_b])
        return np.hstack([stack_a,stack_b])
        


    def value_match(self, a,b,tol,flip_reference=False):
        """
        Given two 1D arrays 'a' and 'b': for each value in 'a', find the index of the closest value in 'b' that is within 'tol', and vice versa.
        The same correspondence must occur in both directions for it to be valid
        params:
            a (1D array): first array (considered the 'reference' array by default)
            b (1D array): second array
            tol (float): maximum difference allowed between two values for them to be considered a correspondence
            print_results (bool): print the results showing the (index, value) correspondence for every value in the reference array
            flip_reference (bool): if True, the reference array is flipped (a->b, b->a)
        returns:
            (cors_a, cors_b) (tuple)
            where:
                cors_a (1D array): same size as 'a', where each value is either: the index of the corresponding value in 'b', or -1 if no correspondence 
                cors_b (1D array): same size as 'b', where each value is either: the index of the corresponding value in 'a', or -1 if no correspondence 
        """
        # a = a.reshape(-1,1)
        # b = b.reshape(-1,1)
        range_a = np.arange(len(a))
        range_b = np.arange(len(b))

        ab = np.apply_along_axis(find_closest, axis=1, arr=a.reshape(-1,1), values=b, tol=tol)
        ba = np.apply_along_axis(find_closest, axis=1, arr=b.reshape(-1,1), values=a, tol=tol)
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

        else:
            print('no correspondences found for array \'a\' of size {} and array \'b\' of size {}'.format(len(a),len(b)))

        # if print_results:
        #     smaller_array = a if len(a)<=len(b) else b
        #     percent = 100*float(len(cors)/len(smaller_array))
        #     print('{} of {} possible correspondences found ({:.2f}%) using tolerance {} for arrays of length {} and {}'
        #         .format(len(cors),len(smaller_array), percent,tol,len(a),len(b)))
        #     name1,name2 = ('a','b')
        #     if flip_reference:
        #         name1,name2 = name2,name1
        #         a,b = b,a
        #         cors_a,cors_b = cors_b,cors_a
        #     for i in range(len(a)):
        #         diff = b[cors_a[i]] - a[i] if cors_a[i]>=0 else None
        #         if diff is not None:
        #             print('(index,value) {}: ({}, {:.04f}) ---> {}: ({},{:.04f}) diff: {:.6f}'.format(name1,i,a[i],name2,cors_a[i],b[cors_a[i]],diff))
        #         else:
        #             print('(index,value) {}: ({}, {:.04f}) ---> no match, abs diff > {:.3f}'.format(name1,i,a[i],tol))
        
            
        return cors_a, cors_b
    
# if __name__=='__main__':
#     a = np.linspace(0,5,5)
#     b = np.linspace(0,5,69) + np.random.random(69)*0.2
#     value_match(a,b,0.84, print_results=True,flip_reference=True)
#     cors_a, cors_b = value_match(a,b,0.84, print_results=True,flip_reference=False)

#     print(b)
#     valid_b = b[cors_b>=0]
#     print(cors_b)
#     print(b[cors_b>=0])