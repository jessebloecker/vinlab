#!/usr/bin/env python 
import numpy as np

class TrajectoryEval():
    def __init__(self, trajectory):
        """
        evaluate sequences within a trajectory
        """
        
        self.min = {}
        self.minval   = {}
        self.max = {}  
        self.maxval = {}
        self.norms = {}
        self.normalized_magnitudes = {}

        vec_attrs = {'pos':trajectory.translation.pos,
                     'vel':trajectory.translation.vel,
                     'acc':trajectory.translation.acc,
                     'body_vel':trajectory.body_vel,
                     'body_acc':trajectory.body_acc,
                     'angvel':trajectory.rotation.angvel,
                     'angacc':trajectory.rotation.angacc,
                     'body_angvel':trajectory.rotation.body_angvel,
                     'body_angacc':trajectory.rotation.body_angacc}
        
        for k, v in vec_attrs.items():
            norms = np.linalg.norm(v,axis=1)
            # norms[0:2] = norms[10:12]
            self.min[k] = np.argmin(norms)
            self.minval[k] = np.min(norms)
            self.max[k] = np.argmax(norms)
            self.maxval[k] = np.max(norms) 
            self.norms[k] = norms #check for outliers
            self.normalized_magnitudes[k] = (norms - self.minval[k])/(self.maxval[k]-self.minval[k])

            # self.norms['angvel'][0:2] = self.norms['angvel'][10:12]

    # def get_extrema(self):
    #     """
    #     if norms are None, get norms first
    #     get extrema of all components of the trajectory
    #     returns:
    #         indices: dictionary of tuples (min,max) corresponding to index numbers
    #         values: dictionary of tuples (min,max) corresponding to values
    #         (the keys in both dictionaries are the same: 'pos','vel','acc','body_vel' etc..)
    #     """
    #     for k, v in vec_attrs.items():
    #         norms = np.linalg.norm(v,axis=1) #only do this once, store result in traj.translation.norms.vel or something
    #                                             # order: None - norms - extrema - normalized_magnitudes 
    #                                             #                   (unitvecs)
    #                                             # normalized here is different than unit vector
    #         self.norms
    #         index_min = np.argmin(norms) 
    #         index_max = np.argmax(norms)
    #         indices[k] = (index_min, index_max)
            
    #         value_min = np.min(norms)
    #         value_max = np.max(norms)
    #         values[k] = (value_min, value_max)
    #         self.extrema['k'] = Extrema(index_min, value_min, index_max, value_max)
        
        
    #     return indices, values