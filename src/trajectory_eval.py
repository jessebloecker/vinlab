#!/usr/bin/env python 
import numpy as np
from scipy.spatial.transform import Rotation

class TrajectoryEval():
    def __init__(self, trajectory):
        """
        evaluate sequences within a trajectory
        """
        self.min = {}
        self.minval = {}
        self.max = {}  
        self.maxval = {}
        self.norms = {}
        self.normalized_norms = {}
        self.rmse_pos = None
        self.rmse_angle = None

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
            self.add_sequence(k,v)
          
        
    def add_sequence(self,name,seq):
        norms = np.linalg.norm(seq,axis=1) if seq.ndim > 1 else seq
        self.min[name] = np.argmin(norms)
        self.minval[name] = np.min(norms)
        self.max[name] = np.argmax(norms)
        self.maxval[name] = np.max(norms) 
        self.norms[name] = norms #check for outliers
        self.normalized_norms[name] = (norms - self.minval[name])/(self.maxval[name]-self.minval[name])

            
class TrajectoryError():
    def __init__(self, target, reference='ground_truth'):
        #enforce that reference has greater or equal n
        n = target.n
        assert(n==reference.n)
        self.target = target.id
        self.reference = reference.id
        self.timestamps = 0 #time
        #for every time stamp in the reference, find the closest time stamp in the target, discard the rest, n may change (warn if it does)


        #target and reference must have the same number of time stamps
        self.pos = target.translation.pos - reference.translation.pos #cant do any of this until timestamp matching is done
        R = reference.rotation.rot.as_matrix().swapaxes(1,2)@target.rotation.rot.as_matrix()
        rot = Rotation.from_matrix(R)
        self.rot = rot
        self.angle = np.linalg.norm(rot.as_rotvec(),axis=1)
        self.rmse_pos = np.sqrt(np.mean(self.pos**2))
        self.rmse_angle = np.rad2deg(np.sqrt(np.mean(self.angle**2)))
        # self.vel = target.translation.vel - reference.translation.vel #enforce that the reference must have greater or equal n
        # self.acc = target.translation.acc - reference.translation.acc
        # self.body_vel = target.body_vel - reference.body_vel
        # self.body_acc = target.body_acc - reference.body_acc
        # self.angvel = target.rotation.angvel - reference.rotation.angvel
        # self.angacc = target.rotation.angacc - reference.rotation.angacc
        # self.body_angvel = target.rotation.body_angvel - reference.rotation.body_angvel
        # self.body_angacc = target.rotation.body_angacc - reference.rotation.body_angacc