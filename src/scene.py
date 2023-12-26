#!/usr/bin/env python

import numpy as np
import yaml
from trajectory import Trajectory
from sensor_platform import SensorPlatform
from features import PointSet, Plane
from scipy.linalg import block_diag
import sys


class Scene():
    def __init__(self, config_file, trajectory, platform=None, features=None):
        """
        A Scene instance contains a trajectory, sensor platform, and/or a set of features.
        """
        self.config_file = config_file
        self.trajectory = trajectory
        self.platform = platform
        self.features = features

    def find(self, _id):
        """
        find trajectory, platform, or feature by id
        """
        _all = self.trajectory + self.platform + self.features
        for i in _all:
            if i.id == frame_id:
                return i
        else:
            raise ValueError('frame_id: \'{}\' not found on platform \'{}\''.format(frame_id,self.id))

    @classmethod
    def config(cls, config_file):
        """
        Initialize a Scene instance from a config file
        """
        config = yaml.load(open(config_file),Loader=yaml.FullLoader)
        trajectory = Trajectory.config(config['trajectory'])
        platform = SensorPlatform.config(config['platform'])
        cf = config['features']
        features = []
        for i in range(len(cf)):
            feat_type = cf[i]['type']
            if feat_type == 'point_set':
                f = PointSet.config(cf[i])
            elif feat_type == 'plane':
                f = Plane.config(cf[i])
            else:
                raise ValueError('invalid feature type: \'{}\' \n'.format(feat_type)+
                                 'supported feature types are: \'point_set\',\'plane\'')
            features.append(f)

        return cls(config_file, trajectory, platform, features)
    

    def get_feature_measurements(self, feats_id, frame_id, start=0.0):
        """
        project a particular set of features in the scene into a particular camera in the scene
        """
        features = self.features
        platform = self.platform
        trajectory = self.trajectory
        try:
            feats = [f for f in features if f.id==feats_id][0]
        except IndexError:
            raise ValueError('feats_id: \'{}\' not found'.format(feats_id,frame_id))
        cam = platform.find(frame_id)
        rate = cam.rate

        if frame_id == platform.base_frame:
            cam_traj = trajectory
        else:
            tf = cam.R, cam.pos
            cam_traj = trajectory.subsample(rate,start=start).transfer(*tf)

        
        n = cam_traj.n #number of poses
        m = feats.n #number of features
        K = cam.K #camera matrix
        assert feats.points.shape == (m,3)

        gPgc = cam_traj.translation.pos
        gRc = cam_traj.rotation.R
        assert gPgc.shape == (n,3)

        # test cases
        #works
        # cRg = gRc[7].T
        # gPgf = feats.points
        # cPgf = (cRg@gPgf.T).T

        # works
        # cRg_block = block_diag(*gRc[7:9,:,:]).T
        # gPgf = feats.points
        # gPgf_repeat = np.tile(feats.points.T, (2,1))
        # cPgf = (cRg_block@gPgf_repeat).T
        # cPgf_3d = (cRg_block@gPgf_repeat).T.reshape(m,2,3).swapaxes(0,1) 

        cRg_block = block_diag(*gRc).T
        gPgf = feats.points
        gPgf_repeat = np.tile(gPgf.T, (n,1))
        cPgf = (cRg_block@gPgf_repeat).T.reshape(m,n,3).swapaxes(0,1) # n x m x 3, where each layer (time value) is all m features (m x 3 array) in the camera frame at that time
        cPcf = cPgf - gPgc.reshape(n,1,3) # n x m x 3 - n x 1 x 3 = n x m x 3 

        cPcf_stack = cPcf.reshape(n*m,3) # n*m x 3
        z_all = cPcf_stack[:,2].reshape(n*m,1) # n*m x 1
        div_z = np.divide(cPcf_stack,z_all) # n*m x 3 (all points in camera frame at all times, divided by their z values, may contain NaNs)
        
        # apply camera matrix to get pixel coordinates
        # proj_all = np.nan_to_num(K@div_z.T).T.reshape(n,m,3).astype(np.int32)
        proj_all = np.nan_to_num(K@div_z.T).T.astype(np.int32)

        width = 620
        height = 480

        # conditions for valid points 
        z_positive = z_all.flatten() > 0
        x_in_frame = np.logical_and(proj_all[:,1]>=0, proj_all[:,1]<width)
        y_in_frame =  np.logical_and(proj_all[:,0]>=0, proj_all[:,0]<height)
        valid = np.all(np.vstack((z_positive,x_in_frame,y_in_frame)).T,axis=1) #true only if all conditions are true

        #boolean masks
        valid_mask = np.tile(valid,(3,1)).T
        invalid_mask = np.logical_not(valid_mask)

        # proj = np.copy(proj_all)
        proj = proj_all
        proj[invalid_mask] = -1
        # print('valid_mask: \n', valid_mask)

        # output 2n x m array: each pair or rows corresponds to a time value, and each column corresponds to a feature, value of -1 indicates invalid (out of frame or behind camera)
        out = proj[:,0:2].reshape(n,m,2).swapaxes(1,2).reshape(2*n,m).astype(np.int16)
        # out = proj[:,0:2].reshape(n,m,2).swapaxes(1,2)

        return out



