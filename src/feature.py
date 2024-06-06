#!/usr/bin/env python 

import numpy as np
from geometry_utils import random_point_set, planar_point_set
from color_helper import ColorHelper
import sys
import logging
from config_utils import check_keys, ConfigurationError

class Feature():
    def __init__(self, feature_id, feature_type, frame_id='global', color=None):
        self.id = feature_id
        self.type = feature_type
        self.frame_id = frame_id
        self.color = color

    @classmethod
    def config(cls, config):
        config, config_mode = check_keys(config, 'feature', context='features')
        config['feature_id'] = config.pop('id')
        
        if config_mode == {'points'}:
            config['feature_type'] = 'points'
            return PointSet.config(config)
        
        elif config_mode == {'planar_points'}:
            config['feature_type'] = 'planar_points'
            return PlanarPointSet.config(config)
        
        elif config_mode == {'random_points'}:
            config['feature_type'] = 'random_points'
            return RandomPointSet.config(config)
        
        elif config_mode == {'plane'}:
            raise NotImplementedError('plane configuration not implemented')

class PointSet(Feature):
    def __init__(self, points, colors=None, **kwargs): #kwargs: feature_id, feature_type, frame_id='global', color=None
        super().__init__(**kwargs)
        self.points = {self.frame_id : points}
        n = len(points)
        if colors is None: #if array of colors is not provided
            color = self.color #apply the uniform color to all points
            ch = ColorHelper('rgb',scale=255)
            if color is None: #if uniform color not provided as Feature attribute, default to rainbow
                self.colors = ch.rainbow_sequence(n)
            else:
                color = ch.__dict__[self.color.upper()]
                self.colors = np.tile(color,(n,1))
        else:
            if not n == len(colors):
                raise ValueError(self.__class__.__name__+': number of colors provided {{}} does not match number of points ({})'
                                 .format(len(colors),n))
            self.colors = colors
        self.n = n

    @classmethod
    def config(cls, config):
        #no keys to check, should just be a list of points
        points = np.array(config.pop('points')).astype(np.float32)
        return cls(points, **config)
    

class PlanarPointSet(PointSet):
    def __init__(self, center, normal, radius, **kwargs): 
        super().__init__(**kwargs)  #kwargs: points,color, feature_id, feature_type, frame_id='global'
        self.center = center
        self.normal = normal
        self.radius = radius

    @classmethod
    def config(cls, config):
        config_planar, config_mode = check_keys(config.pop('planar_points'), 'planar_points', context='feature')
        center = np.array(config_planar['center']).astype(np.float32)
        radius = float(config_planar['radius'])
        normal = np.array(config_planar['normal']).astype(np.float32)
        if config_mode == {'grid_spacing'}:
            config['points'] = planar_point_set(center,normal,radius,grid_spacing=float(config_planar['grid_spacing']))
        elif config_mode == {'num'}:
            config['points'] = planar_point_set(center,normal,radius,num=int(config_planar['num']))
        return cls(center, normal, radius, **config)

class RandomPointSet(PointSet):
    def __init__(self, center, radius, **kwargs):
        super().__init__(**kwargs)
        self.center = center
        self.radius = radius
    
    @classmethod
    def config(cls, config):
        config_random = check_keys(config.pop('random_points'), 'random_points', context='feature')[0]
        center = np.array(config_random['center']).astype(np.float32)
        radius = float(config_random['radius'])
        num = int(config_random['num'])
        config['points'] = random_point_set(center,radius,num)
        return cls(center, radius, **config)
    

class Plane(Feature):
    def __init__(self, center, normal, radius):
        super().__init__(**kwargs)
        self.center = center
        self.normal = normal
        self.radius = radius

    @classmethod
    def config(cls, config):
        raise NotImplementedError('plane configuration not implemented')
