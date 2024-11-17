from trajectory_group import TrajectoryGroup
from feature import Feature
from sensor_platform import SensorPlatform
from config_utils import check_keys, ConfigurationError
import numpy as np
import geometry_utils


class SensorMeasurements():
    def __init__(self, measurement_times, indices, is_upsampled):
       self.upsampled = is_upsampled # if true, indicies represent indicies of original trajectory data (keypoints) in the upsampled trajectory
                                     # if false, indicies represent indicies at which measurements were taken in the original trajectory
       self.times = measurement_times
       self.indices = indices

       self.n = len(measurement_times)
       self.dt = (measurement_times[-1]-measurement_times[0])/self.n
       self.rate = 1./self.dt

class CameraMeasurements(SensorMeasurements):
    def __init__(self,  trajectory, camera, features):
        resampled_trajectory, indices = trajectory.downsample(camera.rate)
        resampled_cam_trajectory = resampled_trajectory.transfer(camera.rot,camera.pos)
        super().__init__(resampled_trajectory.times, indices, False) #assign self.times self.indeces
        # feature_positions = self.get_feature_positions(resampled_trajectory, features)
        pos = resampled_cam_trajectory.translation.pos.values
        rot = resampled_cam_trajectory.rotation.rot
        points = features.points['global']
        
        cPcf = geometry_utils.get_point_positions(pos,rot,points) #feature positions in camera frame at all times
        #ifCpcf is already known at the full trajectory rate, then we can just apply the mask to get the measurement times
        measurements = self.get_projectionss(cPcf, camera)
        self.values = measurements

    def get_projectionss(self, points, camera):
        """
        points: (n,m,3) points in camera frame
        camera: vinlab Camera object
        """
        K = camera.intrinsics
        width = camera.width
        height = camera.height
        noise_std_dev = camera.noise_std_dev

        cPcf = points # n x m x 3 TODO: cPcf should be all times, then apply mask to get measurement times
        n = cPcf.shape[0] #number of poses
        m = cPcf.shape[1] #number of features

        cPcf_stack = cPcf.reshape(n*m,3) # n*m x 3
        z_all = cPcf_stack[:,2].reshape(n*m,1) # n*m x 1
        xy1 = np.nan_to_num(np.divide(cPcf_stack,z_all)) # n*m x 3 (all points in camera frame at all times, divided by their z values)
        xy = xy1[:,0:2] # n*m x 2

        # #apply distortion if distortion coefficients were provided
        # if camera.distortion is not None: # using 'is not None' instead of '!= None' doesn't work here for some reason
        #     coefficients = camera.distortion
        #     xy = self.radtan_distort(xy,coefficients)
        # xy1[:,0:2] = xy
        proj = (K@xy1.T).T

        #add gusian random noise with standard deviation of 1 pixel
        noise = np.random.normal(0,noise_std_dev,proj.shape)
        proj += noise
        proj.astype(np.int32)

        assert proj.shape == (n*m,3)

        # boolean masks for valid points 
        z_positive = z_all.flatten() > 0
        x_in_frame = np.logical_and(proj[:,0]>=0, proj[:,0]<width)
        y_in_frame =  np.logical_and(proj[:,1]>=0, proj[:,1]<height)
        valid = np.all(np.vstack((z_positive,x_in_frame,y_in_frame)).T,axis=1) #true only if all conditions are true

        valid_mask = np.tile(valid,(3,1)).T
        invalid_mask = np.logical_not(valid_mask)
        proj[invalid_mask] = -1

        # output 2n x m array: each pair or rows corresponds to a time value
        # and each column corresponds to a feature, value of -1 indicates invalid (out of frame or behind camera)
        # need to convert from n*m x 2 to 2n x m by doing a 'block transpose', so in the end we have a 
        # vertical stack of 2 x m arrays to form a 2n x m array:  (n*m x 2) --> (n x m x 2) --> (2n x 2 x m) --> (2n x m)
        meas_n_m_2 = proj[:,0:2].reshape(n,m,2)
        assert meas_n_m_2.shape == (n,m,2)
    
        return meas_n_m_2


    def valid(self,index):
        #return two arrays: valid ids and valid values
    
        values = self.values[index]
        num = len(values)
        valid = values[:,0]>=0 #any row containing -1 is invalid

        valid_values = values[valid]
        valid_ids = np.arange(num)[valid]
        return valid_ids, valid_values
        
        
class IMUMeasurements(SensorMeasurements):
    def __init__(self,  trajectory, imu):
        #assume for now that imu mesurements are not faster than full trajectory rate
        resampled_trajectory, indices = trajectory.downsample(imu.rate)
        super().__init__(resampled_trajectory.times, indices, False)

        w = resampled_trajectory.rotation.body_angvel.values #true angular velocity
        a = resampled_trajectory.body_acc.values #true acceleration

        gyro_nd = imu.gyro_noise_density
        gyro_rw = imu.gyro_random_walk
        accel_nd = imu.accel_noise_density
        accel_rw = imu.accel_random_walk

        gyro_measurements = w + self.generate_imu_noise(gyro_rw,gyro_nd)
        accel_measurements = a + self.generate_imu_noise(accel_rw,accel_nd)

        self.values = np.hstack((gyro_measurements,accel_measurements))
        self.values_info = '{} array: gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z'.format(self.values.shape)

    def generate_imu_noise(self,rw,nd):
        n = self.n
        dt = self.dt
        rand1 = np.random.normal(0,nd,(n,3))
        rand2 = np.random.normal(0,nd,(n,3))
        walk = rw * np.sqrt(dt) * np.cumsum(rand1,axis=0)
        white_noise = rand2 * nd / np.sqrt(dt) 
        return walk + white_noise

class VelocityMeasurements(SensorMeasurements):
    pass

    

