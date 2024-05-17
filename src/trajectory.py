#!/usr/bin/env python 
import numpy as np
from translation_trajectory import TranslationTrajectory, BSpline
from rotation_trajectory import RotationTrajectory
from geometry_utils import as_scipy_rotation, rotation_align_axis
from array_utils import q_conjugate
from row_vector_array import RowVectorArray


from trajectory_eval import TrajectoryEval, TrajectoryError

class Trajectory():
    """
    Initialize with TranslationTrajectory and RotationTrajectory objects 
    or from arrays of positions and rotations
    """
    def __init__(self, translation, rotation, _id='traj', is_reference=False):
        self.id = _id
        self.is_reference = is_reference
        self.global_frame = 'global'
        self.moving_frame = 'imu0'
        self.translation = translation
        self.rotation = rotation
        
        # get values from translation component: TODO: check if only rotation component is provided
        self.dur = translation.dur
        self.t = translation.t
        self.dt = translation.dt
        self.rate = translation.rate
        self.n = translation.n

        #Compute body-frame velocities and accelerations
        R = rotation.rot.as_matrix()
        vel = translation.vel.values
        acc = translation.acc.values
        n = translation.n
        self.body_vel = RowVectorArray((R.swapaxes(1,2)@vel.reshape(n,3,1)).reshape(n,3))
        self.body_acc = RowVectorArray((R.swapaxes(1,2)@acc.reshape(n,3,1)).reshape(n,3))
        
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
        pos = self.translation.pos.values
        R = self.rotation.rot.as_matrix()
        traj_rate = self.rate

        if rate <= 0:
            print(self.__class__.__name__+': subsample rate {:.1f} is <= 0, using trajectory rate {:.1f}'.format(rate,traj_rate))
            rate = traj_rate

        t = self.t
        dt = self.dt
        dt_sub = 1./rate

        if interpolate:
            #TODO
            pass
        else:
            if rate > traj_rate:
                print(self.__class__.__name__+': subsample rate {:.1f} exceeds or matches trajectory rate {:.1f}, using trajectory rate'.format(rate,traj_rate))
                step = 1
            else:
                step = round(dt_sub/dt)
            s = np.argmin(np.abs(t-start)) #index of closest matching time
            pos_sub = pos[s::step,:]
            R_sub = R[s::step,:,:]
            t_sub = t[s::step]

        traj_sub = Trajectory.from_arrays(pos_sub,R_sub,t=t_sub)
        return traj_sub
    
    def transform(self,rotation,translation):
        """
        transform the trajectory to another rigidly attached frame
        params:
            rotation: rotation matrix (axes of the new frame expressed in the current body frame) or euler angles (fixed-axis xyz)
            translation: position vector (origin of the new frame expressed in the current body frame)
        returns:
            traj: new Trajectory object corresponding to the new frame
        """
        #current position and rotation arrays
        R = self.rotation.rot.as_matrix()
        pos = self.translation.pos.values
        t = self.t #timestamps

        #static transform given
        _rot = as_scipy_rotation(rotation).as_matrix()
        R_static = _rot.reshape(1,3,3) #reshape for broadcasting
        pos_static = translation

        #transform each pose
        R_new = R@R_static # (n x 3 x 3) @ (1 x 3 x 3) = n x 3 x 3
        pos_new = pos + np.squeeze(R@pos_static.reshape(1,3,1)) 
       
        traj = Trajectory.from_arrays(pos_new,R_new,t=t)
        return traj
        
    def to_file(self,path,jpl=True,delimiter=' '):
        from datetime import datetime
        now = datetime.now().strftime("%Y.%m.%d %H:%M:%S")
        """
        write trajectory to text file
        """
        convention = ('hamilton','jpl')[int(jpl)]
        _id = self.id
        t = self.t.reshape(-1,1)
        p = self.translation.pos
        rot = self.rotation.rot
        q = q_conjugate(rot.as_quat()) if jpl else rot.as_quat()
        header = 'timestamp(s), px, py, pz, qx, qy, qz, qw [INFO: file created: {}, trajectory id: {}, quaternion convention: {}]'.format(now,_id,convention)
        data = np.hstack((t,p,q))

        np.savetxt(path,data,fmt='%0.6f',delimiter=delimiter,header=header)
        
    
    @classmethod
    def from_file(cls, path, jpl=True, _id='traj'):
        """
        load trajectory from text file
        """
        print('loading trajectory from file: {}, jpl: {}'.format(path,jpl))
        data = np.loadtxt(path,delimiter=',',skiprows=0)
        t = data[:,0]
        p = data[:,1:4]
        q = q_conjugate(data[:,4:]) if jpl else data[:,4:]  

        traj = Trajectory.from_arrays(p,q,t=t,_id=_id)
        return traj

        
    @classmethod
    def from_arrays(cls, pos,rot,*, t=None, dur=None,vel=None,acc=None, _id='traj'):
        """
        initialize Trajectory object from arrays
        if rot is None, initialize with identity rotation
            id rot is single value, initialize with constant rotation
        if pos is None, initialize with zero translation
            if pos is single value, initialize with constant translation
        """
        translation = TranslationTrajectory(pos, t, dur)
        rotation = RotationTrajectory(rot, t, dur)

        return cls(translation,rotation,_id=_id)
    
    @classmethod
    def config(cls, config):
        _id = config['id']
        if 'file' in config.keys():
            return cls.from_file(config['file'],jpl=config['jpl'],_id=_id)
            # if 'align' in config.keys():
            #     align_type = config['align']
            #     if align_type == 'best_fit':
            #         translation_traj, rotation_traj = utils.align_trajectory(translation_traj,rotation_traj)
            # return cls.from_arrays(translation_traj,rotation_traj,_id)
        
        translation = config['translation']
        rotation = config['rotation']
        if translation['type'] == 'bspline':
            translation_traj = BSpline(**translation['bspline'])
        elif translation['type'] == 'from_file':
            pass

        if rotation['type'] == 'align_axis':
            params = rotation['align_axis']
            flip = params['flip']
            axis = params['axis']
            grounded_axis = params['grounded_axis']
            _with = {'pos': translation_traj.pos.values, 'vel':translation_traj.vel.values, 'acc': translation_traj.acc}[params['with']]
            rots = rotation_align_axis(axis,_with,grounded_axis,flip)
            rotation_traj = RotationTrajectory(rots,dur=translation_traj.dur)

        return cls(translation_traj,rotation_traj,_id)
    
class TrajectoryGroup():
    """
    group of related Trajectory objects - e.g. the ground truth and the estimate
    """
    def __init__(self, trajectories, reference):
        
        self.reference = reference
        self.trajectories = trajectories
        self.n = len(trajectories)
        self.get_error()
    
    def get_error(self):
        """
        get error between reference and target trajectories
        """
        error = {}
        trajectories = self.trajectories
        ref = self.reference
        for k,v in trajectories.items():
            if k != ref:
                # print(v.n)
                error = TrajectoryError(v,trajectories[ref])
                v.eval.add_sequence('error_pos',error.pos)
                v.eval.add_sequence('error_angle',error.angle)
                v.eval.rmse_pos = error.rmse_pos
                v.eval.rmse_angle = error.rmse_angle
        # return error

    @classmethod
    def config(cls, config):
        reference = config['reference']
        ct = config['trajectories']
        trajectories = {}
        for i in ct:
            t = Trajectory.config(i)
            trajectories[t.id] = t
        return cls(trajectories,reference)
