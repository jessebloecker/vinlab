#!/usr/bin/env python 

import numpy as np
from scipy.spatial.transform import Rotation
import motion_utils as utils
from geometry_utils import as_scipy_rotation
from config_utils import config_transform

class SensorPlatform():
    """
    note: changed file name and class name to sensor_platform 
    so it doesn't conflict with built-in module called 'platform'
    Initialize sensor platform
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
        find sensor or frame by id
        """
        # print('finding frame_id: {}'.format(frame_id))
        for s in [self.sensors, self.body_frames]:
            for k,v in s.items():
                # print('k: {} v.id: {} '.format(k,v.id))
                if v.id == frame_id:
                    return v
        raise ValueError(self.__class__.__name__+': frame_id: \'{}\' not found on platform \'{}\''.format(frame_id,self.id))
        
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
                    rot  = utils.as_scipy_rotation(_0R1@_1R2)
                    pos = _0P01 + _0R1 @ _1P12
                    v.rot, v.pos  = rot,pos

    @classmethod
    def config(cls, config):
        platform_id = config['id']
        base_frame = config['base_frame']
        cb = config['body_frames']
        body_frames = {}
        for i in range(len(cb)):
            body_frames[cb[i]['id']] = BodyFrame.config(cb[i])

        cs = config['sensors']
        sensors = {}
        ci = [i for i in cs if i['type'].lower()=='imu']
        num_imus = len(ci)
        
        for i in range(num_imus):
            # print('configuring imu {}'.format(ci[i]['id']))
            sensors[ci[i]['id']] = Imu.config(ci[i])
        cc = [i for i in cs if i['type'].lower()=='camera']
        num_cameras = len(cc)
        
        for i in range(num_cameras):
            # print('configuring camera {}'.format(cc[i]['id']))
            sensors[cc[i]['id']]= Camera.config(cc[i])
        
        return cls(platform_id, base_frame, sensors, body_frames)

class Sensor():
    def __init__(self, sensor_id, enable_measurements=True, rate=0 , rot=None, pos=None, resolve_transform_from=None):
        self.id = sensor_id
        self.enable_measurements = enable_measurements
        self.rate = float(rate)
        self.pos = np.array([0.,0.,0.]) if pos is None else pos
        self.rot = Rotation.from_matrix(np.eye(3)) if rot is None else rot
        self.resolve_transform_from = resolve_transform_from
        # self.R =  Rotation.from_matrix(np.eye(3)) if rot is None else rot.as_matrix()
    

    @classmethod
    def subclass_config(cls,config):
        """
        common configuration for all subclasses
        return param tuple, instead of initializing this class,
        to be used to initialize one of the sub-classes
        maybe this is a better way for all of them...
       
        params = Sensor.subclass_config(config)
        sensor = Sensor(**params)
        """
        sensor_id = config['id']
        enable_measurements = config['enable_measurements']
        rate = config['rate']
        rot,pos = config_transform(config['transform'])

        if 'from' in config['transform'].keys():
            resolve_transform_from = config['transform']['from']
        else: 
            resolve_transform_from = None
        return sensor_id, enable_measurements, rate, rot, pos, resolve_transform_from

class Camera(Sensor):
    """
    Pinhole camera model with z-axis aligned with optical axis
    """
    def __init__(self, sensor_id, enable_measurements, rate, rot, pos, resolve_transform_from, height, width, intrinsics, distortion=None):
        super().__init__(sensor_id, enable_measurements, rate, rot, pos, resolve_transform_from)
        self.type= 'camera'
        self.width = width
        self.height = height
        fx,fy,cx,cy = intrinsics
        # k1,k2,p1,p2 = distortion
        self.K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
        self.distortion = np.array(distortion)

    @classmethod
    def config(cls, config):
        # sensor_id, enable_measurements, rate, rot, pos, resolve_transform_from  = Sensor.subclass_config(config)
        sensor_config = Sensor.subclass_config(config)
        # vinlab_config.param_check(cls,config)
        # if any value is None (recursively), remove it from the config
        cc = config['camera']
        keyset = set(cc.keys())
        required_keyset = set(['height','width','intrinsics'])
        allowed_keyset = required_keyset.union(set(['distortion']))
        if not required_keyset <= keyset:
            raise Exception(cls.__name__+': missing required parameter(s): {}'.format(', '.join(required_keyset-(keyset&required_keyset))))
        width = int(cc['width'])
        height = int(cc['height'])
        intrinsics = np.array(cc['intrinsics']).astype(np.float32)

        if 'distortion' in cc.keys():
            distortion = np.array(cc['distortion']).astype(np.float32) 
        else:
            distortion = None
        
        # return cls(sensor_id, enable_measurements, rate, rot, pos, resolve_transform_from, height, width, intrinsics, distortion)
        return cls(*sensor_config, height, width, intrinsics, distortion)
    

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
    def __init__(self, sensor_id, enable_measurements, rate, rot, pos, resolve_transform_from, gyro_noise=0.01, gyro_bias=0.01, accel_noise=0.01, accel_bias=0.01):
        super().__init__(sensor_id, enable_measurements, rate, rot, pos, resolve_transform_from)
        self.type = 'imu'
        self.gyro_noise = gyro_noise
        self.gyro_bias = gyro_bias
        self.accel_noise = accel_noise
        self.accel_bias = accel_bias

      
    @classmethod
    def config(cls, config):
        sensor_config  = Sensor.subclass_config(config)
        ci = config['imu']
        gyro_noise = float(ci['gyro_noise'])
        gyro_bias = float(ci['gyro_bias'])  
        accel_noise = float(ci['accel_noise'])
        accel_bias = float(ci['accel_bias'])

        return cls(*sensor_config, gyro_noise, gyro_bias, accel_noise, accel_bias)
    


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
        frame_id = config['id']
        rot,pos = config_transform(config['transform'])

        return cls(frame_id, rot, pos)


