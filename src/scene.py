#!/usr/bin/env python

import numpy as np
import yaml
from trajectory import Trajectory, TrajectoryGroup
from sensor_platform import SensorPlatform
from features import PointSet, Plane
from scipy.linalg import block_diag

np.set_printoptions(precision=4, suppress=True)

class Scene():
    def __init__(self, config_file, trajectory_group, platform=None, features=None):
        """
        A scene contains a trajectory group, sensor platform, and/or a set of features.
        Feature positions in each camera fame at all times and their corresponding measurements are
        computed and stored as attributes of the scene.   
        """

        self.config_file = config_file
        self.trajectory_group = trajectory_group
        self.platform = platform
        self.features = features

        if platform is not None:
            self.trajectory_group.moving_frame = platform.base_frame

        #combine all features
        all_points = np.vstack([f.points['global'] for f in features.values() if isinstance(f,PointSet)])
        all_colors = np.vstack([f.colors for f in features.values() if isinstance(f,PointSet)])
        self.features['all'] = PointSet('all', 'point_set', all_points, all_colors)

        #for each camera, get all features expressed in the camera frame at all times
        for k,v in platform.sensors.items():
            if v.type == 'camera':
                # print('computing feature positions for camera: ', k)
                points = self.get_feature_positions(k)
                self.features['all'].points[k] = points

        #compute measurements for all sensors that have measurements enabled
        measurements = {}  
        cam_ids = [i for i in features['all'].points.keys() if i != 'global']
        for i in cam_ids:
            if platform.sensors[i].enable_measurements:
                # print('computing feature measurements for camera: ', i)
                m = self.get_feature_measurements(i)
                measurements[i] = m
            elif v.type == 'imu':
                pass
        self.measurements = measurements

        # self.print_scene_info()
    
    @classmethod
    def config(cls, config_file):
        """
        Initialize a Scene instance from a config file
        """
        config = yaml.load(open(config_file),Loader=yaml.FullLoader)
        # config = utils.remove_nonetype(yaml.load(open(config_file),Loader=yaml.FullLoader))
        trajectory_group = TrajectoryGroup.config(config['trajectory_group'])
        platform = SensorPlatform.config(config['platform'])


        cf = config['features']
        features = {}
        for i in range(len(cf)):
            feat_type = cf[i]['type']
            f = PointSet.config(cf[i])
            features[f.id] = f

        return cls(config_file, trajectory_group, platform, features)
    

    def get_feature_positions(self, frame_id, feats_id='all', start=0.0):
        """
        transform all features into frame_id frame at all times
        update scene.features[feats_id].points[frame_id] with result
        """
        ref = self.trajectory_group.reference
        trajectory = self.trajectory_group.trajectories[ref]
        platform = self.platform
        features = self.features[feats_id]
    
        #given frame id must be a camera
        cam = platform.find(frame_id)
        if not cam.type == 'camera':
            raise ValueError('frame_id: \'{}\' is not a camera'.format(frame_id))
        
        rate = cam.rate
        height = cam.height
        width = cam.width

        if frame_id == platform.base_frame:
            cam_traj = trajectory.subsample(rate,start=start)
        else:
            tf = cam.rot, cam.pos
            # print('\'{}\' is not the base frame, using tf:\n R:{}\np:{}'.format(frame_id,cam.rot.as_matrix(),cam.pos))
            cam_traj = trajectory.subsample(rate,start=start).transform(*tf)
        # print('orig trajectory: {} poses, cam trajectory: {} poses'.format(trajectory.n,cam_traj.n))
        
        n = cam_traj.n #number of poses
        m = features.n #number of features
        K = cam.K #camera matrix

        gPgf = features.points['global'] # m x 3
        gPgc = cam_traj.translation.pos # n x 3
        gRc = cam_traj.rotation.rot.as_matrix() # n x 3 x 3

        assert gPgf.shape == (m,3)
        assert gPgc.shape == (n,3)

        #reshape for broadcasting
        gPgf = gPgf.reshape(1,m,3).swapaxes(1,2)
        gPgc = gPgc.reshape(n,3,1) 
        assert gPgf.shape == (1,3,m)
        assert gPgc.shape == (n,3,1)

        cRg = np.swapaxes(gRc,1,2) # n x 3 x 3 (transpose each rotation matrix)
        assert cRg.shape == (n,3,3)

        cPcg = (-cRg@gPgc).swapaxes(1,2) # n x 3 x 3 @ n x 3 x 1 = n x 3 x 1 -> swapaxes -> n x 1 x 3
        assert cPcg.shape == (n,1,3)

        cPgf = (cRg@gPgf).swapaxes(1,2) # n x 3 x 3 @ 1 x 3 x m = n x 3 x m -> swapaxes -> n x m x 3
        assert cPgf.shape == (n,m,3)

        cPcf = cPcg + cPgf # n x m x 3 - n x 1 x 3 = n x m x 3
        assert cPcf.shape == (n,m,3)
   
        return cPcf
    
    def get_feature_measurements(self, frame_id, feats_id='all', start=0.0):
        """
        project a particular set of features in the scene into a particular camera in the scene
        """
        platform = self.platform
        features = self.features

        cPcf = features[feats_id].points[frame_id] # n x m x 3
        n = cPcf.shape[0] #number of poses
        m = cPcf.shape[1] #number of features
        K = platform.sensors[frame_id].K #camera matrix
        width = platform.sensors[frame_id].width
        height = platform.sensors[frame_id].height

        cPcf_stack = cPcf.reshape(n*m,3) # n*m x 3
        z_all = cPcf_stack[:,2].reshape(n*m,1) # n*m x 1
        xy1 = np.nan_to_num(np.divide(cPcf_stack,z_all)) # n*m x 3 (all points in camera frame at all times, divided by their z values)
        xy = xy1[:,0:2] # n*m x 2
        # assert(np.allclose(xy1[:,2],np.ones(n*m)))

        #get vector of radii for each point
        r = np.linalg.norm(xy,axis=1)
        k1, k2, p1, p2 = platform.sensors[frame_id].distortion
        f = 1 + k1*r**2 + k2*r**4 #distortion factor
        xy1 = (f*xy1.T).T
        # print('\n\n xy1: {} \n\n\n'.format(xy1))
        # assert(np.allclose(xy1[:,2],r))

        proj = (K@xy1.T).T.astype(np.int32)
        assert proj.shape == (n*m,3)

        # boolean masks for valid points 
        z_positive = z_all.flatten() > 0
        x_in_frame = np.logical_and(proj[:,0]>=0, proj[:,0]<width)
        y_in_frame =  np.logical_and(proj[:,1]>=0, proj[:,1]<height)
        valid = np.all(np.vstack((z_positive,x_in_frame,y_in_frame)).T,axis=1) #true only if all conditions are true

        valid_mask = np.tile(valid,(3,1)).T
        invalid_mask = np.logical_not(valid_mask)
        proj[invalid_mask] = -1

        # output 2n x m array: each pair or rows corresponds to a time value
        # and each column corresponds to a feature, value of -1 indicates invalid (out of frame or behind camera)
        # need to convert from n*m x 2 to 2n x m by doing a 'block transpose', so in the end we have a 
        # vertical stack of 2 x m arrays to form a 2n x m array:  (n*m x 2) --> (n x m x 2) --> (2n x 2 x m) --> (2n x m)
        out = proj[:,0:2].reshape(n,m,2).swapaxes(1,2).reshape(2*n,m).astype(np.int16) 
        assert out.shape == (2*n,m)

        return out
    
    def radial_distortion(self, r, k1,k2):
        return (1 + k1*r**2 + k2*r**4)
    
    def get_imu_measurements(self, frame_id):
        """
        return measurements for a particular imu
        """
        # get acceleration value in the imu frame 
        # get angular velocity in the imu frame
        # apply noise
        # measurement = [w_x, w_y, w_z, a_x, a_y, a_z]
        pass

    def get_correspondences(frame1,time1,frame2=None,time2=None):

        # scene.features.points['cam0'] = n x m x 3 3d points in cam0 frame at all times
        # scene.features.colors = m x 3 colors of all points
        # scene.measurements['cam0'] = 2n x m 2d points in cam0 frame at all times
        # scene.get_correspondences('3d2d', cam0',t1) = 5 x m
        # scene.get_correspondences('2d2d', cam0',t1,cam0',t1+1) = 4 x m

        pass

        
    def print_scene_info(self,output_file=None):
        """
        print a description of the scene to the console or to a file
        """
        PURPLE = "\033[0;35m"
        BOLDYELLOW = "\033[1;33m"
        YELLOW = "\033[0;33m"
        BOLD = "\033[1m"
        END = "\033[0m"
        indent='    '
        config_file = self.config_file
        print(BOLDYELLOW+'============================ SCENE INFO ============================'+END)
        print(BOLDYELLOW+'Config: '+END+'{}'.format(self.config_file)+END)

        tg = self.trajectory_group
        ids = ', '.join([i for i in tg.trajectories.keys()])
        ref = ['',tg.reference]
        print(('{}'+BOLDYELLOW+'Trajectories: '+END+'\'{}\'').format(indent,ids))
        for t in tg.trajectories.values():
            _id = '[REF] '+t.id if t.id==tg.reference else t.id
            print(('{}'+YELLOW+'{}'+END).format(indent*2,_id))
            print(('{}'+PURPLE+'poses'+END+': {} '+PURPLE+'duration'+END+': {:0.3f} '+PURPLE+'rate'+END+': {:0.3f} '+PURPLE+'dt'+END+': {:0.3f}')
            .format(indent*2,t.n,t.dur,t.rate,t.dt))
            # print(('{}'+PURPLE+'rmse_pos'+END+': {} '+PURPLE+'rmse_angle'+END+': {:0.3f} ')
            # .format(indent*2,t.eval.rmse_pos,t.eval.rmse_angle))
        p = self.platform
        cameras = [v for k,v in p.sensors.items() if v.type == 'camera']
        imus = [v for k,v in p.sensors.items() if v.type == 'imu']
        body_frames = [v for k,v in p.body_frames.items()]
        print(('{}'+BOLDYELLOW+'Platform: '+END+'\'{}\' (cameras: {}  imus: {}  body_frames: {})')
              .format(indent,p.id,len(cameras),len(imus),len(body_frames)))
        for c in cameras:
            _id = c.id+' [BASE FRAME]' if c.id==p.base_frame else c.id
            print(('{}'+YELLOW+'{}'+END+' (camera) ')
                .format(indent*2,_id))
            print(('{}'+PURPLE+'rate'+END+': {:0.2f}  '+PURPLE+'height'+END+': {}  '+PURPLE+'width'+END+': {}')
                  .format(indent*2,c.rate,c.height,c.width))
            print(('{}'+PURPLE+'pos'+END+': [{:8.3f} {:8.3f} {:8.3f} ]').format(indent*2,*c.pos))
            print(('{}'+PURPLE+'rpy'+END+': [{:8.3f} {:8.3f} {:8.3f} ](deg)').format(indent*2,*c.rot.as_euler('xyz',degrees=True)))
            print(('{}'+PURPLE+'calibration'+END+': [{:5.0f} {:5.0f} {:5.0f}]').format(indent*2,*c.K[0]))
            print(('{}     [{:5.0f} {:5.0f} {:5.0f}]').format(indent*4,*c.K[1]))
            print(('{}     [{:5.0f} {:5.0f} {:5.0f}]').format(indent*4,*c.K[2]))
            # print(('{}'+PURPLE+'distortion'+END+':[{:8.3f} {:8.3f} {:8.3f} {:8.3f}] (rad-tan)\n').format(indent*2,*c.distortion))
        for i in imus:
            _id = i.id+' [BASE FRAME]' if i.id==p.base_frame else i.id
            print(('{}'+YELLOW+'{}'+END+' (imu) ')
                .format(indent*2,_id))
            print(('{}'+PURPLE+'rate'+END+': {:0.2f}').format(indent*2,i.rate))
            print(('{}'+PURPLE+'pos'+END+': [{:8.3f} {:8.3f} {:8.3f} ]').format(indent*2,*i.pos))
            print(('{}'+PURPLE+'rpy'+END+': [{:8.3f} {:8.3f} {:8.3f} ](deg)').format(indent*2,*i.rot.as_euler('xyz',degrees=True)))
            print(('{}'+PURPLE+'gyro_noise'+END+': {:8.5f}  '+PURPLE+'gyro_bias'+END+': {:8.5f}').format(indent*2, i.gyro_noise,i.gyro_bias))
            print(('{}'+PURPLE+'accel_noise'+END+': {:8.5f}  '+PURPLE+'accel_bias'+END+': {:8.5f}\n').format(indent*2, i.accel_noise,i.accel_bias))
        for b in body_frames:
            _id = b.id+' [BASE FRAME]' if b.id==p.base_frame else b.id
            print(('{}'+YELLOW+'{}'+END+' (body frame) ')
                .format(indent*2,_id))
            print(('{}'+PURPLE+'pos'+END+': [{:8.3f} {:8.3f} {:8.3f} ]').format(indent*2,*b.pos))
            print(('{}'+PURPLE+'rpy'+END+': [{:8.3f} {:8.3f} {:8.3f} ](deg)\n').format(indent*2,*b.rot.as_euler('xyz',degrees=True)))


        f = self.features
        print(('{}'+BOLDYELLOW+'Features: '+END+'{} total points').format(indent,len(f['all'].points['global'])))
        for k,v in f.items():
            if isinstance(v,PointSet) and not k == 'all':
                print(('{}'+YELLOW+'{:<10s}'+END+': {:<18s} '+PURPLE+'n'+END+': {}').format(indent*2,k, v.type,v.n))

        print(BOLDYELLOW+'===================================================================='+END)
            