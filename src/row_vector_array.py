#!/usr/bin/env python 
import numpy as np

class RowVectorArray():
    """
    n x m array of row vectors, their norms, and basic statistics
    """
    def __init__(self, arr):
        if arr.ndim != 2:
            raise ValueError(self.__class__.__name__+': array must be 2d, got {}d array of shape {}'.format(arr.ndim, arr.shape))
        self.values = arr.astype(np.float32)
        self.n = len(arr)
        norms = np.linalg.norm(arr,axis=1)
        self.norms = norms
        self.min = np.argmin(norms) #index of minimum norm
        self.minval = np.min(norms) #value of minimum norm
        self.max = np.argmax(norms) #index of maximum norm
        self.maxval = np.max(norms) #value of maximum norm
        self.mean = np.mean(norms) #value of mean norm
        self.normalized_norms = (norms - self.minval)/(self.maxval-self.minval) #norms remapped to [0,1]
