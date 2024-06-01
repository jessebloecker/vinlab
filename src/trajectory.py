#!/usr/bin/env python 
import numpy as np
from translation_trajectory import TranslationTrajectory, BSpline
from rotation_trajectory import RotationTrajectory
from geometry_utils import as_scipy_rotation, rotation_align_axis
from array_utils import q_conjugate
from row_vector_array import RowVectorArray
from trajectory_eval import TrajectoryEval
from trajectory_error import TrajectoryError
from config_utils import check_keys

class Trajectory():
    """
    Initialize with TranslationTrajectory and RotationTrajectory objects 
    or from arrays of positions and rotations
    """
    def __init__(self, translation, rotation, _id='traj', super_id=None, is_reference=False):
        self.id = _id
        self.super_id = super_id #id of the parent trajectory (trajectory that this one is based on) if it exists
        self.sub_ids = None #list of ids of the trajectories that this one is based on
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
                         if False, ssceneubsample at closest possible rate with existing poses/timestamps
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
    
    def transfer(self,rotation,translation):
        """
        should call this something else, like 'transfer', and have another method that 
        transforms the trajectory to another fixed frame

        transform the trajectory to another rigidly attached frame - apply relative transform at every time step - express result in global frame
        return the new trajectory expressed in the global frame
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
    
    def rotate_about(self, about, rotation):
        """
        rotate the entire trajectory about a fixed global point
        """
        raise NotImplementedError

    def shift(self,translation):
        """
        shift the trajectory by a fixed global translation
        """
        raise NotImplementedError
        
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
    def from_file(cls, path, _format, time_unit, jpl=None, _id='traj'):
        """
        load trajectory from text file
        """
        print('loading trajectory from file: {}'.format(path))
        req = ['t','x','y','z','qx','qy','qz','qw']
        delimiter = ' ' if ' ' in list( _format) else ','
        f = [i for i in _format.split(delimiter) if i in req+['-']] #format list, filter out invalid characters/extra spaces
        if len(f) < 8:
            raise ValueError('invalid format string - must contain {}'.format(' '.join(req)))
        time_units = {'s':1,'ms':1e-3,'us':1e-6,'ns':1e-9}
        if time_unit not in time_units.keys():
            raise ValueError('invalid time unit - must be s, ms, us, or ns')

        
        data = np.loadtxt(path, delimiter=',',skiprows=0)
        initial_time = data[0,0]
        t = (data[:,f.index('t')] - initial_time)*time_units[time_unit]
        # print(t)
        p = data[:,[f.index('x'),f.index('y'),f.index('z')]]
        q =  data[:,[f.index('qx'),f.index('qy'),f.index('qz'),f.index('qw')]] 
        if jpl is None:
            raise ValueError('quaternion convention not specified for id: \'{}\': set jpl to True or False'.format(_id))
        else:
            q = q_conjugate(q) if jpl else q
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
        config_mode, config = check_keys('trajectory',config)
        _id = config['id']
        if config_mode=={'translation','rotation'}:
         
            ct = config['translation']
            translation_traj = TranslationTrajectory.config(ct) #returns TranslationTrajectory object if independent, otherwise returns the config 
            dependent_translation = True if translation_traj == ct else False
       
            cr = config['rotation'] 
            rotation_traj = RotationTrajectory.config(cr) #returns RotationTrajectory object if independent, otherwise returns the config 
            dependent_rotation = True if rotation_traj == cr else False
            
            if 'constant' in cr.keys() and 'constant' in ct.keys():
                raise ConfigurationError('translation and rotation cannot both be constant. Define a fixed frame instead.')
                    
            if dependent_translation:
                raise NotImplementedError
            
            if dependent_rotation:
                if 'align_axis' in cr.keys():
                    align_axis = cr['align_axis']
                    align_axis_config  = check_keys('align_axis',cr['align_axis'])[1]
                    vec_config = check_keys('vec', align_axis_config.pop('vec'))[1]
                    if 'current_trajectory' in vec_config:
                        name = vec_config['current_trajectory']
                        vec_array = {
                            'pos': translation_traj.pos.values, #swap the name with actual array it refers to
                            'vel': translation_traj.vel.values, 
                            'acc': translation_traj.acc.values,
                            'centroid': translation_traj.bearing_vectors(translation_traj.centroid)}[name]
                    elif 'point' in vec_config:
                        vec_array = np.array(vec_config['point']) - translation_traj.pos.values
                    if 'negate' in vec_config:
                        vec_array = -vec_array
                    
                    rots = rotation_align_axis(**align_axis_config, vec=vec_array)
                    rotation_traj = RotationTrajectory(rots,dur=translation_traj.dur)
                if 'constant' in cr.keys():
                    raise NotImplementedError

        elif config_mode=={'file'}:
            cf = config['file']
            path, fmt, time_unit, jpl = cf['path'], cf['format'], cf['time_unit'], cf['jpl']
            return cls.from_file(path, fmt, time_unit, jpl, _id)
        
        elif config_mode=={'subsample_interpolate'}:
                config_mode, options, config = check_keys('subsample_interpolate',config.keys())
                #depends on another existing trajectory: construct empty trajectory with super_id
                return config
        
        if 'modify' in config.keys():
            raise NotImplementedError
        

        return cls(translation_traj,rotation_traj,_id)
    
    
    def subsample_interpolate(self, order=3, control_point_rate=10, control_point_num=None, super_id=None):
        """
        fit a bspline to the current trajectory, return a new trajectory

        subsample the trajectory at any rate or number of points:
        (enforce both end points)

        determine the span time: amount of time b/w (order+1) control points

        determine res (at least the number of points in the original trajectory)
        
        pin endpoints by duplicating
        """

        sub_traj = self.subsample(control_point_rate, interpolate=False)
        control_points = sub_traj.translation.pos.values
        span_time = sub_traj.dt
        l = len(control_points)
        _res = int((self.n/(l-order))) #approximate number of points per span to match original trajectory
        # res = _res*4 #compute at higher resolution, then subsample
        res = _res #compute at higher resolution, then subsample
        translation = BSpline(res=res, order=order, span_time=span_time, control_points=control_points)  
        n,t = translation.n, translation.t
        
        rotation = RotationTrajectory(np.zeros((n,3)),t=t)
        # new_traj = Trajectory(translation,rotation,_id=self.id+'_bspline').subsample(self.rate)
        new_traj = Trajectory(translation,rotation,_id=self.id+'_bspline')
        
        
        return new_traj
    
