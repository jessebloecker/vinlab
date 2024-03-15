#!/usr/bin/env python

import numpy as np
import yaml
from trajectory import Trajectory
from sensor_platform import SensorPlatform
from features import PointSet, Plane
from scipy.linalg import block_diag

np.set_printoptions(precision=4, suppress=True)

class Scene():
    def __init__(self, config_file, trajectory, platform=None, features=None):
        """
        A Scene instance is initialized with a trajectory, sensor platform, and/or a set of features,
        Then, feature positions in each camera fame at all times and their corresponding measurements are
        computed and stored as attributes of the scene.   
        """
        self.config_file = config_file
        self.trajectory = trajectory
        self.platform = platform
        self.features = features

        #combine all features
        all_points = np.vstack([f.points['global'] for f in features.values() if isinstance(f,PointSet)])
        all_colors = np.vstack([f.colors for f in features.values() if isinstance(f,PointSet)])
        self.features['all'] = PointSet('all',all_points, all_colors)

        #for each camera, get all features expressed in the camera frame at all times
        for k,v in platform.sensors.items():
            if v.type == 'camera':
                print('computing feature positions for camera: ', k)
                points = self.get_feature_positions(k)
                self.features['all'].points[k] = points

        #compute measurements for all sensors that have measurements enabled
        measurements = {}  
        cam_ids = [i for i in features['all'].points.keys() if i != 'global']
        for i in cam_ids:
            if platform.sensors[i].enable_measurements:
                print('computing feature measurements for camera: ', i)
                m = self.get_feature_measurements(i)
                measurements[i] = m
            elif v.type == 'imu':
                pass
        self.measurements = measurements

    @classmethod
    def config(cls, config_file):
        """
        Initialize a Scene instance from a config file
        """
        config = yaml.load(open(config_file),Loader=yaml.FullLoader)
        trajectory = Trajectory.config(config['trajectory'])
        platform = SensorPlatform.config(config['platform'])

        cf = config['features']
        features = {}
        for i in range(len(cf)):
            feat_type = cf[i]['type']
            if feat_type == 'point_set':
                f = PointSet.config(cf[i])
            elif feat_type == 'plane':
                f = Plane.config(cf[i])
            else:
                raise ValueError('invalid feature type: \'{}\' \n'.format(feat_type)+
                                 'supported feature types are: \'point_set\',\'plane\'')
            features[f.id] = f

        # 'around trajectory'
        # for p in trajectory.translation.pos:
        #     n = 1
        #     r_max = 1
        #     v = np.random.uniform(-1,1,(n,3))
        #     u = (v.T/np.linalg.norm(v,axis=1)) #random unit vectors
        #     r = np.cbrt(np.random.uniform(0,r_max**3,n)) #random scales, prevent clustering at center
        #     c = np.array(p).astype(np.float32)
        #     points.append(np.multiply(u,r).T.astype(np.float32) + np.tile(c,(n,1)))
    
        # features['test'] = PointSet('test',np.vstack(points))

        return cls(config_file, trajectory, platform, features)
    

    def get_feature_positions(self, frame_id, feats_id='all', start=0.0):
        """
        transform all features into frame_id frame at al times
        update scene.features[feats_id].points[frame_id] with result
        """
        trajectory = self.trajectory
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
            tf = cam.R, cam.pos
            print('\'{}\' is not the base frame, using tf:\n R:{}\np:{}'.format(frame_id,cam.R,cam.pos))
            cam_traj = trajectory.subsample(rate,start=start).transfer(*tf)
        # print('orig trajectory: {} poses, cam trajectory: {} poses'.format(trajectory.n,cam_traj.n))
        
        n = cam_traj.n #number of poses
        m = features.n #number of features
        K = cam.K #camera matrix

        gPgf = features.points['global'] # m x 3
        gPgc = cam_traj.translation.pos # n x 3
        gRc = cam_traj.rotation.R # n x 3 x 3

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
        normalized_proj = np.nan_to_num(np.divide(cPcf_stack,z_all)) # n*m x 3 (all points in camera frame at all times, divided by their z values)

        proj = (K@normalized_proj.T).T.astype(np.int32)
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
    