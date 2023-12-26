#!/usr/bin/env python 

import numpy as np
from translation_trajectory import TranslationTrajectory, BSpline
from rotation_trajectory import RotationTrajectory
import motion_utils as utils

class Trajectory():
    """
    Initialize with TranslationTrajectory and RotationTrajectory objects 
    or from arrays of positions and rotations
    """
    def __init__(self, translation, rotation, traj_id='traj'):

        self.id = traj_id
        self.frame = 'global'
        self.body_frame = 'body'
        self.translation = translation
        self.rotation = rotation

        # get values from translation component: TODO: check if only rotation component is provided
        self.n = translation.n
        self.dur = translation.dur
        self.t = translation.t
        self.dt = translation.dt
        self.rate = translation.rate

        R = rotation.R
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

    def subsample(self,rate,start=0.0,interpolate=False):
        """
        subsample trajectory at a given rate and starting time
        params:
            rate: sampling rate in Hz (if <=0, use all samples)
            start: starting time in seconds
            interpolate: if True, enforce exact rate, linearly interpolate poses between samples
                         if False, subsample at closest possible rate with existing poses/timestamps
        returns:
            traj_sub: new Trajectory object at a subsampled rate
        """
    
        pos = self.translation.pos
        R = self.rotation.R
        orig_rate = self.rate
    
        if rate <= 0:
            print(self.__class__.__name__+': subsample rate {:.1f} is <= 0, using original rate {:.1f}'.format(rate,orig_rate))
            rate = orig_rate

        t = self.t
        dt = self.dt
        dt_sub = 1./rate

        if interpolate:
            #TODO
            pass
        else:
            if rate > orig_rate:
                print(self.__class__.__name__+': subsample rate {:.1f} exceeds or matches original rate {:.1f}, using original rate'.format(rate,orig_rate))
                step = 1
            else:
                step = round(dt_sub/dt)
            s = np.argmin(np.abs(t-start)) #index of closest matching time
            pos_sub = pos[s::step,:]
            R_sub = R[s::step,:,:]
            t_sub = t[s::step]

        # print('subsampled rate: requested: {} actual: {}'.format(rate,1/np.median(np.diff(t_sub))))
        traj_sub = Trajectory.from_arrays(pos_sub,R_sub,t=t_sub)
        return traj_sub
    
    def transfer(self,rotation,translation):
        """
        transfer the trajectory to another rigidly attached frame
        params:
            rotation: rotation matrix (axes of the new frame expressed in the current body frame) or euler angles (fixed-axis xyz)
            translation: position vector (origin of the new frame expressed in the current body frame)
        returns:
            traj: new Trajectory object corresponding to the new frame
        """
        _rot = utils.as_scipy_rotation(rotation).as_matrix()
        R_static = _rot.reshape(1,3,3)
        t = translation

        pos = self.translation.pos
        R = self.rotation.R
     
        R_new = R@R_static # (n x 3 x 3) @ (1 x 3 x 3) = n x 3 x 3
        pos_new = pos + np.squeeze(R@t.reshape(1,3,1)) 

        traj = Trajectory.from_arrays(pos_new,R_new,t=self.t)
        return traj
        
    def to_file(self,dst,format='default'):
        """
        write trajectory to text file
        """
        return NotImplemented
    def from_file(self):
        """
        load trajectory from text file
        """
        return NotImplemented

    @classmethod
    def from_arrays(cls, pos,rot,*, dur=None,t=None,vel=None,acc=None,name=None):
        """
        initialize Trajectory object from arrays
        if rot is None, initialize with identity rotation
            id rot is single value, initialize with constant rotation
        if pos is None, initialize with zero translation
            if pos is single value, initialize with constant translation
        """
        
        translation = TranslationTrajectory(pos, t, dur)
        rotation = RotationTrajectory(rot, t, dur)

        return cls(translation,rotation)
    
    @classmethod
    def config(cls, config):
        #if dict, initialize from dict
        #if file, isolate only the trajectory config

        traj_id = config['id']
        translation = config['translation']
        rotation = config['rotation']
        
        if translation['type'] == 'bspline':
            traj = BSpline(**translation['bspline'])
        elif translation['type'] == 'from_file':
            pass

        if rotation['type'] == 'align_axis':
            params = rotation['align_axis']
            flip = params['flip']
            axis = params['axis']
            grounded_axis = params['grounded_axis']
            _with = {'pos': traj.pos, 'vel':traj.vel, 'acc': traj.acc}[params['with']]
            rots = utils.rotation_align_axis(axis,_with,grounded_axis,flip)
            rotation_traj = RotationTrajectory(rots,dur=traj.dur)

        return cls(traj,rotation_traj,traj_id)
    
