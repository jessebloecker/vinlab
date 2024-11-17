#!/usr/bin/env python 
import numpy as np
from scipy.spatial.transform import Rotation
from config_utils import check_keys, config_transform, ConfigurationError

class StaticFrame():
    """
    Static frame from which trajectories or features may be defined"
    """
    def __init__(self, frame_id, rot=None, pos=None, resolve_transform_from=None):
        self.id = frame_id
        self.sensors = None #TODO: allow for static cameras or static IMUs. this sensors attribute should be exactly like SensorPlatform.sensors
        self.pos = np.array([0.,0.,0.]) if pos is None else pos
        self.rot = Rotation.from_matrix(np.eye(3)) if rot is None else rot
        self.resolve_transform_from = resolve_transform_from
    
    @classmethod
    def config(cls, config):
        cs = check_keys(config, 'static_frame', context='scene')[0] #configure a camera or IMU object
        _id = cs.pop('id')
        if _id.lower() == 'global':
            raise ConfigurationError('StaticFrame: id \'global\' always exists by default, choose a different name')
        cs['frame_id'] = _id
        rot, pos, _from = config_transform(cs.pop('transform'))
        cs['rot'] = rot
        cs['pos'] = pos
        cs['resolve_transform_from'] = _from
        return cls(**cs)
