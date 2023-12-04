#!/usr/bin/env python 

import numpy as np
from scipy.spatial.transform import Rotation
from translation_trajectory import TranslationTrajectory, BSpline
from rotation_trajectory import RotationTrajectory
# from bspline import Bspline
from scipy.linalg import block_diag
import motion_utils as utils
class Trajectory():
    """
    Initialize with TranslationTrajectory and RotationTrajectory objects 
    or from arrays of positions and rotations
    """
    def __init__(self, translation, rotation, name='traj'):

        self.name = name
        self.frame = 'global'
        self.body_frame = 'body'
        self.translation = translation
        self.rotation = rotation


        
        n = self.translation.n
        dur = self.translation.dur
        t = np.linspace(0,dur,n)
        self.n = self.translation.n
        self.dur = self.translation.dur
        self.t = t

        R = self.rotation.R
        vel = self.translation.vel
        acc = self.translation.acc
        self.body_vel = utils.in_frame(R,vel)
        self.body_acc = utils.in_frame(R,acc)
        
    def normalize_timestamps(self):
        """
        normalize timestamps by subtracting initial time stamp from all time stamps
        """
        if self.t is None:
            raise ValueError('No time stamps provided')
        self.t = self.t - self.t[0]

    def subsample(self,rate):
        """
        subsample trajectory at a given rate
        or at the given indices
        or at the given time stamps
        return new Trajectory
        """
        return NotImplemented

    @classmethod
    def from_arrays(cls, pos,rot,*,dur=None,t=None,vel=None,acc=None,name=None):
        """
        initialize Trajectory object from arrays
        if rot is None, initialize with identity rotation
            id rot is single value, initialize with constant rotation
        if pos is None, initialize with zero translation
            if pos is single value, initialize with constant translation
        """
        
        translation = TranslationTrajectory(pos, dur,t)
        rotation = RotationTrajectory(rot, dur,t)

        return cls(translation,rotation)
    
    @classmethod
    def config(cls, config):
        _config = config

        name = config['name']
        _translation = config['translation']
        _rotation = config['rotation']
        
        if _translation['type'] == 'bspline':
            translation = BSpline(**_translation['bspline'])
        elif _translation['type'] == 'from_file':
            pass

        if _rotation['type'] == 'align_axis':
            params = _rotation['align_axis']
            _flip = params['flip']
            _axis = params['axis']
            _grounded_axis = params['grounded_axis']
            _with = {'pos': translation.pos, 'vel':translation.vel, 'acc': translation.acc}[params['with']]
            rots = utils.rotation_align_axis(_axis,_with,_grounded_axis,_flip)
            rotation = RotationTrajectory(rots,dur=translation.dur)

        return cls(translation,rotation,name)


    
    

