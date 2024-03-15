#!/usr/bin/env python 

import numpy as np
from scipy.spatial.transform import Rotation
import motion_utils as utils
from color_helper import ColorHelper

class Feature():
    def __init__(self, feats_id, lifetime=0, frame_id='global'):
        self.id = feats_id
        self.lifetime = lifetime
        self.frame_id = frame_id
    
class PointSet(Feature):
    def __init__(self, feats_id, points, colors=None):
        super().__init__(feats_id, lifetime=0, frame_id='global')
        self.points = {self.frame_id : points} #should be a dictionary of numpy arrays for each frame
        self.n = len(points)
        if colors is None:
            self.colors = self.assign_colors()
        elif colors.ndim==1:
            self.colors = np.tile(colors[0:3],(self.n,1))
        else:
            self.colors = colors

    def assign_colors(self):
        """
        assign colors to points based on their index
        """
        ch = ColorHelper()
        n = self.n
        colors = ch.rainbow_sequence(n)
        return colors
    
    @classmethod
    def config(cls, config):
        feats_id = config['id']
        if config['type'] == 'point_set':
            keyset = set(config.keys())
            if {'points'} <= keyset:
                points = np.array(config['points']).astype(np.float32)

            if {'center', 'radius', 'num'} <= keyset: #generate 'num' random points, uniformly distributed                                           
                n = config['num']                     #within in a sphere with 'center' and 'radius'
                r_max = config['radius']
                v = np.random.uniform(-1,1,(n,3))
                u = (v.T/np.linalg.norm(v,axis=1)) #random unit vectors
                r = np.cbrt(np.random.uniform(0,r_max**3,n)) #random scales, prevent clustering at center
                c = np.array(config['center']).astype(np.float32)
                points = np.multiply(u,r).T.astype(np.float32) + np.tile(c,(n,1))

            if {'color'} <= keyset:
                ch = ColorHelper('rgba',scale=255)
                colors = ch.__dict__[config['color'].upper()]  
            else:
                colors = None

        return cls(feats_id,points,colors)
    
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
    


