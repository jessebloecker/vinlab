#!/usr/bin/env python 

import numpy as np
from scipy.spatial.transform import Rotation
import motion_utils as utils



class Feature():
    def __init__(self, feats_id, lifetime=0, frame_id='global'):
        self.id = feats_id
        self.lifetime = 0
        self.frame_id = 'global'
        

class PointSet(Feature):
    def __init__(self, feats_id, points):
        super().__init__(feats_id, lifetime=0, frame_id='global')
        self.points = points
        self.n = len(points)

    @classmethod
    def config(cls, config):
        feats_id = config['id']
        if config['type'] == 'point_set':
            keyset = set(config.keys())
            if {'points'} <= keyset:
                points = np.array(config['points']).astype(np.float32)

            if {'center', 'radius', 'num'} <= keyset:
                """
                generate 'num' random points, uniformly distributed 
                within in a sphere with 'center' and 'radius'
                """
                n = config['num']
                r_max = config['radius']
                v = np.random.uniform(0,1,(n,3))
                u = (v.T/np.linalg.norm(v,axis=1)) #random unit vectors
                r = np.cbrt(np.random.uniform(0,r_max**3,n)) #random scales, prevent clustering at center
                c = config['center']
                points = np.multiply(u,r).T.astype(np.float32) 
                norms = np.linalg.norm(points,axis=1)
                # print('r: {}, v: {}, u: {}, points: {}'.format(r.shape,v.shape,u.shape,points.shape))
                # print('mean radius:', np.mean(norms))

        return cls(feats_id,points)
    
class Plane(Feature):
    def __init__(self, center, normal, radius):
        super().__init__(feats_id, lifetime=0, frame_id='global')
        self.center = center
        self.normal = normal
        self.radius = radius

    def config(cls, config):
        feats_id = config['feats_id']
        if config['type '] == 'plane':
            keyset = set(config.keys())
            if {'center', 'normal', 'radius'} <= keyset:
                center = np.array(config['center']).astype(np.float32)
                normal = np.array(config['normal']).astype(np.float32)
                radius = np.array(config['radius']).astype(np.float32)
        return cls(feats_id,center,normal,radius)
    


