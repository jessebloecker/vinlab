#!/usr/bin/env python 

import numpy as np
from scipy.spatial.transform import Rotation
import motion_utils as utils

class SensorPlatform():
    """
    note: changed file name and class name to sensor_platform 
    so it doesn't conflict with built-in module called 'platform'
    Initialize sensor platform
    """
    def __init__(self, platform_id='mono', base_frame='imu', imus=[], cameras=[], body_frames=[]):
        self.id = platform_id
        self.base_frame = base_frame
        self.body_frames = body_frames
        self.cameras = cameras
        self.imus = imus

        #check base_frame
        R = self.find(base_frame).R
        pos = self.find(base_frame).pos
        base_is_identity = np.allclose(R,np.eye(3)) and np.allclose(pos,np.zeros(3))
        if not base_is_identity:
            print('Warning: base_frame \'{}\' transform must be the identity, setting to identity...'.format(self.base_frame))
            self.find(base_frame).R = np.eye(3)
            self.find(base_frame).pos = np.zeros(3)

    def find(self, frame_id):
        """
        find sensor or frame by id
        """
        _all = self.cameras + self.imus + self.body_frames
        for i in _all:
            if i.id == frame_id:
                return i
        else:
            raise ValueError('frame_id: \'{}\' not found on platform \'{}\''.format(frame_id,self.id))

    @classmethod
    def config(cls, config):
        platform_id = config['id']
        base_frame = config['base_frame']
        body_frames = config['body_frames']
        cs = config['sensors']
        cc = [i for i in cs if i['type'].lower()=='camera']
        num_cameras = len(cc)
        cameras = []
        for i in range(num_cameras):
            cameras.append(Camera.config(cc[i]))
        
        ci = [i for i in cs if i['type'].lower()=='imu']
        num_imus = len(ci)
        imus = []
        for i in range(num_imus):
            imus.append(Imu.config(ci[i]))

        return cls(platform_id, base_frame, imus, cameras)

class Sensor():
    def __init__(self, sensor_id, rate=0,R=None,pos=None):
        self.id = sensor_id
        self.rate = float(rate)
        self.pos = np.array([0.,0.,0.]) if pos is None else pos
        self.R =  np.eye(3) if R is None else R
    

    @classmethod
    def param_config(cls,config):
        """
        return param tuple, instead of initializing this class,
        to be used to initialize one of the sub-classes
        maybe this is a better way for all of them...
       
        params = Sensor.param_config(config)
        sensor = Sensor(**params)
        """
        sensor_id = config['id']
        rate = config['rate']
        R,pos = utils.config_transform(config['transform'])

        return sensor_id, rate, R, pos

class Camera(Sensor):
    """
    Pinhole camera model with z-axis aligned with optical axis
    """
    def __init__(self, sensor_id, rate, R, pos, height, width, intrinsics, distortion=None):
        super().__init__(sensor_id, rate, R, pos)
        self.width = width
        self.height = height
        fx,fy,cx,cy = intrinsics
        self.K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
        self.distortion = distortion

    @classmethod
    def config(cls, config):
        sensor_id, rate, R, pos = Sensor.param_config(config)
        cc = config['camera']
        width = int(cc['width'])
        height = int(cc['height'])
        intrinsics = np.array(cc['intrinsics']).astype(np.float32)
        distortion = None
        
        return cls(sensor_id, rate, R, pos, height, width, intrinsics, distortion)
    


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
    def __init__(self, sensor_id, rate, R, pos, gyro_noise=0.01, gyro_bias=0.01, accel_noise=0.01, accel_bias=0.01):
        super().__init__(sensor_id, rate, R, pos)
        self.gyro_noise = gyro_noise
        self.gyro_bias = gyro_bias
        self.accel_noise = accel_noise
        self.accel_bias = accel_bias

      
    @classmethod
    def config(cls, config):
        sensor_id, rate, R, pos = Sensor.param_config(config)
        ci = config['imu']
        gyro_noise = float(ci['gyro_noise'])
        gyro_bias = float(ci['gyro_bias'])  
        accel_noise = float(ci['accel_noise'])
        accel_bias = float(ci['accel_bias'])

        return cls(sensor_id, rate, R, pos, gyro_noise, gyro_bias, accel_noise, accel_bias)
    


    



