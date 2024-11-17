#!/usr/bin/env python 

import numpy as np
from scipy.spatial.transform import Rotation
from geometry_utils import as_scipy_rotation
from config_utils import check_keys, config_transform, ConfigurationError

class SensorPlatform():
    """
    SensorPlatform object - contains all sensors and body frames on a platform
    """
    def __init__(self, platform_id='platform', base_frame='imu', sensors = {}, body_frames=[]):
        self.id = platform_id
        self.base_frame = base_frame
        self.body_frames = body_frames
        self.sensors = sensors
        self.check_base_frame()
        self.resolve_transforms() #resolve all transforms that are not configured with respect to base_frame

    def find(self, frame_id):
        """
        Find sensor or body frame by id, raise ConfigurationError if not found
        """
        # print('finding frame_id: {}'.format(frame_id))
        for s in [self.sensors, self.body_frames]:
            for k,v in s.items():
                # print('k: {} v.id: {} '.format(k,v.id))
                if v.id == frame_id:
                    return v
        raise ConfigurationError(self.__class__.__name__+': frame_id: \'{}\' not found on platform \'{}\''.format(frame_id,self.id))
        
    def check_base_frame(self):
        base = self.find(self.base_frame)
        rot, pos = base.rot, base.pos
        base_is_identity = np.allclose(rot.as_matrix(),np.eye(3)) and np.allclose(pos,np.zeros(3))
        if not base_is_identity:
            print(self.__class__.__name__+': base_frame \'{}\' transform must be identity, setting to identity...'.format(self.base_frame))
            base.rot = as_scipy_rotation(np.eye(3))
            base.pos = np.zeros(3)

        
    def resolve_transforms(self):
        """
        - resolve transform of a sensor or body frame whose transform is not configured with respect to base frame
        - only one layer of composition is allowed (i.e. parent frame must be defined with respect to base frame)
        """
        base_frame = self.base_frame
        for s in [self.sensors, self.body_frames]:
            for k,v in s.items():
                if v.resolve_transform_from is None:
                    v.resolve_transform_from = base_frame
                    print(self.__class__.__name__+': \'{}\' transform parent frame not given, setting to base frame \'{}\''.format(v.id,base_frame))
                if v.resolve_transform_from != base_frame:
                    parent_frame = self.find(v.resolve_transform_from)
                    child_frame = v
                    if parent_frame.resolve_transform_from != base_frame:
                        raise ValueError(self.__class__.__name__+': \'{}\' transform not resolved since parent frame \'{}\' is not defined with respect to base frame \'{}\''
                                            .format(child_frame.id,parent_frame.id, base_frame))
                    _0R1  = parent_frame.rot.as_matrix()
                    _0P01 = parent_frame.pos
                    _1R2 = child_frame.rot.as_matrix()
                    _1P12 = child_frame.pos
                    rot  = as_scipy_rotation(_0R1@_1R2)
                    pos = _0P01 + _0R1 @ _1P12
                    v.rot, v.pos  = rot,pos

    @classmethod
    def config(cls, config):
        config = check_keys(config, 'platform', context='scene')[0]
        platform_id = config['id']
        base_frame = config['base_frame']

        sensors={}
        if 'sensors' in config.keys():
            cs = config['sensors']
            for i in range(len(cs)):
                s = Sensor.config(cs[i])
                sensors[s.id] = s


        body_frames = {}
        if 'body_frames' in config.keys():
            cb = config['body_frames']
            for i in range(len(cb)):
                b = BodyFrame.config(cb[i])
                body_frames[b.id] = b
        
        return cls(platform_id, base_frame, sensors, body_frames)

class Sensor():
    def __init__(self, sensor_id, enable_measurements=True, rate=0 , rot=None, pos=None, resolve_transform_from=None, time_offset=0.0):
        self.id = sensor_id
        self.enable_measurements = enable_measurements
        self.rate = float(rate)
        self.dt = 1./rate if rate > 0 else 0
        self.pos = np.array([0.,0.,0.]) if pos is None else pos
        self.rot = Rotation.from_matrix(np.eye(3)) if rot is None else rot
        self.resolve_transform_from = resolve_transform_from
        # self.R =  Rotation.from_matrix(np.eye(3)) if rot is None else rot.as_matrix()

    @classmethod
    def config(cls,config):
        config, config_mode = check_keys(config, 'sensor', context='sensors')
        config['sensor_id'] = config.pop('id')

        if 'transform' in config.keys():
            rot, pos, _from = config_transform(config.pop('transform'))
            config['rot'] = rot #replace with rotation object
            config['pos'] = pos #replace with the array
            config['resolve_transform_from'] = _from
        else:
            #this is bad...should restrucutre camera and imu initialization
            config['rot'] = Rotation.from_matrix(np.eye(3))
            config['pos'] = np.zeros(3)
            config['resolve_transform_from'] = None

        if config_mode == {'camera'}:
            return Camera.config(config)

        elif config_mode == {'imu'}:
            return Imu.config(config)
    
class Camera(Sensor):
    """
    Pinhole camera model with z-axis aligned with optical axis
    """
    def __init__(self, sensor_id, enable_measurements, rate, rot, pos, resolve_transform_from, time_offset, height, width, intrinsics=None, fov=None, distortion=None, noise_std_dev=0.0):
        super().__init__(sensor_id, enable_measurements, rate, rot, pos, resolve_transform_from)
        self.type = 'camera'
        self.width = int(width)
        self.height = int(height)
        if intrinsics:
            fx,fy,cx,cy = np.array(intrinsics).astype(np.float32)
        elif fov:
            fx = fy = 0.5*width/(np.tan(np.deg2rad(fov)/2))
            cx = width/2
            cy = height/2
        self.K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]]) #should change this to 'intrinsics'
        self.intrinsics = self.K
        self.distortion = np.array(distortion) if distortion is not None else None
        self.noise_std_dev = float(noise_std_dev)

    @classmethod
    def config(cls, config):
        cc = check_keys(config.pop('camera'), 'camera', context='sensor')[0]
        if 'distortion' in cc.keys():
            distortion = np.array(cc['distortion']).astype(np.float32) 
            cc['distortion'] = distortion
        return cls(**config,**cc)
    

    def get_image_lines(self, line):
        """
        get_image a line object onto the camera sensor
        """
        pass

    def get_image_planes(self, plane):
        """
        get_image a plane object onto the camera sensor
        """
        pass


class Imu(Sensor):
    """
    Imu object
    """
    def __init__(self, sensor_id, enable_measurements, rate, rot, pos, resolve_transform_from, time_offset, gyro_noise_density=0.01, gyro_random_walk=0.01, accel_noise_density=0.01, accel_random_walk=0.01):
        super().__init__(sensor_id, enable_measurements, rate, rot, pos, resolve_transform_from)
        self.type = 'imu'
        self.gyro_noise_density = float(gyro_noise_density)
        self.gyro_random_walk = float(gyro_random_walk)
        self.accel_noise_density = float(accel_noise_density)
        self.accel_random_walk = float(accel_random_walk)

    @classmethod
    def config(cls, config):
        ci = check_keys(config.pop('imu'), 'imu', context='sensor')[0]
        return cls(**config,**ci)
    

class BodyFrame():
    """
    BodyFrame object - contains only an id and transform
    """
    def __init__(self, frame_id, rot=None, pos=None, resolve_transform_from=None):
        self.id = frame_id
        self.pos = np.array([0.,0.,0.]) if pos is None else pos
        self.rot = Rotation.from_matrix(np.eye(3)) if rot is None else rot
        self.resolve_transform_from = resolve_transform_from
    @classmethod
    def config(cls, config):
        cb = check_keys(config, 'body_frame', context='body_frames')[0]
        cb['frame_id'] = cb.pop('id')
        if 'transform' in cb.keys():
            rot, pos, _from = config_transform(cb.pop('transform'))
        cb['rot'] = rot
        cb['pos'] = pos
        cb['resolve_transform_from'] = _from
        return cls(**cb)
