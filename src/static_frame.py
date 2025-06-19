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
        self.resolve_transform_from = resolve_transform_from #do not need to retain this, resolution should be done before constructing 
        self.info = None #TODO: add info about where it was resolved from and how it was configured
    @classmethod
    def config(cls, config):
        cs, cs_mode = check_keys(config, 'static_frame', context='scene') #configure a camera or IMU object
        # print('cs {}, cs_mode {}'.format(cs, cs_mode))
        _id = cs['id']
        if _id.lower() == 'global':
            raise ConfigurationError('StaticFrame: id \'global\' always exists by default, choose a different name') #maybe remove this
        if cs_mode == {'transform'}:
            rot, pos, _from = config_transform(cs['transform'])
            # cs['rot'] = rot
            # cs['pos'] = pos
            # cs['resolve_transform_from'] = _from
            return cls(frame_id=_id, rot=rot, pos=pos, resolve_transform_from=_from)
