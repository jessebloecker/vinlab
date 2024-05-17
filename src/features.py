#!/usr/bin/env python 

import numpy as np
from geometry_utils import random_point_set, planar_point_set
from color_helper import ColorHelper
import sys
import logging

class Feature():
    def __init__(self, feats_id, _type, lifetime=0, frame_id='global'):
        self.id = feats_id
        self.type = _type
        self.lifetime = lifetime
        self.frame_id = frame_id
    
class PointSet(Feature):
    def __init__(self, feats_id, _type, points, colors=None):
        super().__init__(feats_id, _type, lifetime=0, frame_id='global')
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
        _id = config['id']
        keyset = set(config.keys())
        required_keysets = {'point_set': set(['points']),
                           'random_point_set': set(['center', 'radius', 'num']),
                           'planar_point_set': set(['center', 'radius', 'normal'])} #also requires either num or grid_spacing
        _type = config['type'].lower()
        allowed_types = required_keysets.keys()
        if _type not in allowed_types:
            print('invalid type \'{}\' for feature id: \'{}\' - valid types are: {} '.format(_type,_id, ', '.join(allowed_types)))
            sys.exit()
        required_keyset = required_keysets[_type]

        if required_keyset <= keyset:
            if _type == 'point_set':
                points = np.array(config['points']).astype(np.float32)
                if {'center'} <= keyset:
                    center = np.array(config['center']).astype(np.float32)
                    points += center
            
            elif _type == 'random_point_set':
                center = np.array(config['center']).astype(np.float32)
                radius = config['radius']
                n = config['num'] 
                points = random_point_set(n,radius,center)

            elif _type == 'planar_point_set':
                radius = np.array(config['radius']).astype(np.float32)
                center = np.array(config['center']).astype(np.float32)
                normal = np.array(config['normal']).astype(np.float32)
                if {'grid_spacing'} <= keyset:
                    grid_spacing = np.array(config['grid_spacing']).astype(np.float32)
                    points = planar_point_set(radius, center, normal,grid_spacing=grid_spacing)
                elif {'num'} <= keyset:
                    n = config['num']
                    points = planar_point_set(radius, center, normal,n)
                else:
                    print('provide either num or grid_spacing for {} \'{}\''.format(_type,_id))
                    sys.exit()
            else:
                logging.error('invalid type \'{}\' for feature id: \'{}\' - valid types are: {} '.format(_type,_id, ', '.join(required_keysets.keys())))
                # print('invalid type \'{}\' for feature id: \'{}\' - valid types are: {} '.format(_type,_id, ', '.join(required_keysets.keys())))
                sys.exit()

        else:
            print('missing or invalid parameters for {} \'{}\' - required: {}'.format(_type,_id, ', '.join(required_keyset)))
            sys.exit()
    
        if {'color'} <= keyset:
            ch = ColorHelper('rgba',scale=255)
            colors = ch.__dict__[config['color'].upper()]  
        else:
            colors = None

        return cls(_id, _type, points, colors)
    
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
    

