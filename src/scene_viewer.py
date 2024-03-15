#!/usr/bin/env python 
import sys
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node   import Node
import numpy as np
import tf2_ros
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, Float32, Int16
from  nav_msgs.msg    import Path
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import  MarkerArray, Marker
import ros_message_helper as rmh
from functools import partial
from trajectory import Trajectory

from scene import Scene
import yaml
from scipy.spatial.transform import Rotation
import motion_utils as utils
from color_helper import ColorHelper
from std_msgs.msg   import *
import tkinter as tk

np.set_printoptions(threshold=sys.maxsize,suppress=True,precision=3,linewidth=100)

class SceneViewer(Node):
    def __init__(self):
        super().__init__('scene_viewer')
        self.declare_params()
        scene_config =  self.get_parameter('scene').get_parameter_value().string_value
        if not scene_config:
            self.get_logger().error('no scene configuration set - provide path to yaml file. \n   usage: \n   '
                                    'ros2 run motion_tools scene_viewer.py --ros-args --params-file path/to/scene_viewer.yaml')
            sys.exit()
        self.scene = Scene.config(scene_config)

        p,s = self.init_pub_sub()
        self.pub = p
        self.sub = s
        self.messages = self.get_messages()

        
        self.playing = False
        # self.playing = True

        slow_rate = self.get_parameter('slow_rate').get_parameter_value().double_value
        fast_rate = self.get_parameter('fast_rate').get_parameter_value().double_value
        self.start_timers(slow_rate,fast_rate)

        self.i = 0
        self.increment = float(1/self.scene.trajectory.n)

        
    def declare_params(self):

        self.declare_parameter('scene','')
        self.declare_parameter('loop',False)
        self.declare_parameter('slow_rate',1.0)
        self.declare_parameter('fast_rate',30.0)
        self.declare_parameter('colors.trajectory.res',100)
        self.declare_parameter('colors.trajectory.wrt','vel')
        self.declare_parameter('colors.trajectory.color_min','green')
        self.declare_parameter('colors.trajectory.color_max','red')
        self.declare_parameter('colors.features','rainbow')
    
    def init_pub_sub(self):
        """
        initialize publishers and subscribers
        """
        traj = self.scene.trajectory
        now = self.get_clock().now().nanoseconds
        
        sub_slider = self.create_subscription(Float32,'/slider/value',self.slider_cb,10)
        subscribers = {}
        publishers = {}
        publishers['index_markers'] = self.create_publisher(MarkerArray,'~/index_markers', 10)
        publishers['transforms'] = tf2_ros.TransformBroadcaster(self)
        publishers['paths'] = {'pos': self.create_publisher(Marker,'~/'+traj.id+'/pos',10)}
        publishers['features'] = self.create_publisher(MarkerArray,'~/features',10)
        publishers['feature_measurements'] = self.create_publisher(Int16MultiArray,'~/feature_measurements',10)
        publishers['feature_colors'] = self.create_publisher(Int16MultiArray,'~/feature_colors',10)
        publishers['camera_infos'] = self.create_publisher(CameraInfo,'~/camera_info',10)
        publishers['mark'] = self.create_publisher(Float32,'~/mark',10)
        publishers['index'] = self.create_publisher(Int16,'~/index',10)
       
        for i in ('vel','acc', 'angvel', 'angacc'):
            publishers['paths'][i] = self.create_publisher(Path,'~/'+traj.id+'/'+i,10)
        return publishers, subscribers
    
    def slider_cb(self,msg):
        n = self.scene.trajectory.n
        self.i = int(msg.data*(n-1))
        self.get_logger().info('slider value: {}, index: {}'.format(msg.data,self.i))
    
    def get_messages(self):
        """
        Convert trajectory to Path messages, list of transforms, and list of marker arrays
        corresponding to each time step in the trajectory
        """
        traj = self.scene.trajectory
        features = self.scene.features
        meas = self.scene.measurements['cam0']
        sensors = self.scene.platform.sensors
        # feature_measurements= meas.swapaxes(1,2).reshape(2*meas.shape[0],meas.shape[1]).astype(np.int16)
        body_frame = traj.body_frame

        messages = {'paths':{'pos': rmh.as_marker_msg(frame_id=traj.frame, scale=[0.05]*3, marker_type=4),
                             'vel': rmh.as_path_msg(frame_id=traj.frame), 
                             'acc': rmh.as_path_msg(frame_id=traj.frame),
                             'angvel': rmh.as_path_msg(frame_id=traj.frame),
                             'angacc': rmh.as_path_msg(frame_id=traj.frame)},
                    # 'index_markers':rmh.as_marker_msg(frame_id=traj.frame, scale=rmh.as_vector3_msg([0.25]*3), marker_type=7),
                    'index_markers': [],
                    'transforms':[], 
                    'mark': Float32(),
                    'index': Int16(),
                    'features':rmh.as_markerarray_msg(frame_id=traj.frame,n=len(features),marker_type=7, scale=[0.05]*3),
                    'camera_infos': [] }
                    # 'feature_measurements':rmh.as_int16multiarray_msg(feature_measurements)}
        
        measurements_msg = Int16MultiArray()
        measurements_msg.layout.dim.append(MultiArrayDimension())
        measurements_msg.layout.dim.append(MultiArrayDimension())
        measurements_msg.layout.dim[0].label = "time"
        measurements_msg.layout.dim[0].size = meas.shape[0]
        measurements_msg.layout.dim[0].stride = meas.shape[0]*meas.shape[1]
        measurements_msg.layout.dim[1].label = "feature"
        measurements_msg.layout.dim[1].size = meas.shape[1]
        measurements_msg.layout.dim[1].stride = meas.shape[1]
        measurements_msg.data = meas.flatten().tolist()
        messages['feature_measurements'] = measurements_msg

        feature_colors = features['all'].colors
        # print ('feature_colors',feature_colors)
        feature_colors_msg = Int16MultiArray()
        feature_colors_msg.layout.dim.append(MultiArrayDimension())
        feature_colors_msg.layout.dim.append(MultiArrayDimension())
        feature_colors_msg.layout.dim[0].label = "feature"
        feature_colors_msg.layout.dim[0].size = feature_colors.shape[0]
        feature_colors_msg.layout.dim[0].stride = feature_colors.shape[0]*3
        feature_colors_msg.layout.dim[1].label = "rgb"
        feature_colors_msg.layout.dim[1].size = 3
        feature_colors_msg.layout.dim[1].stride = 3
        feature_colors_msg.data = feature_colors.flatten().tolist()
        messages['feature_colors'] = feature_colors_msg



        for k,v in sensors.items():
            if v.type == 'camera' and v.enable_measurements:
                camera_info_msg = CameraInfo()
                camera_info_msg.header.frame_id = k
                camera_info_msg.height = v.height
                camera_info_msg.width = v.width
                camera_info_msg.k = v.K.flatten().tolist()
                camera_info_msg.distortion_model = 'plumb_bob'
                camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
                messages['camera_infos'].append(camera_info_msg)


        ch = ColorHelper('rgba',scale=1)
        color_res = self.get_parameter('colors.trajectory.res').get_parameter_value().integer_value
        color_min = ch.__dict__[(self.get_parameter('colors.trajectory.color_min').get_parameter_value().string_value).upper()]
        color_max = ch.__dict__[(self.get_parameter('colors.trajectory.color_max').get_parameter_value().string_value).upper()]
        color_features = ch.__dict__[(self.get_parameter('colors.features').get_parameter_value().string_value).upper()]
        colors = utils.linear_path(color_min, color_max, color_res)
        magnitudes = traj.eval.normalized_magnitudes[self.get_parameter('colors.trajectory.wrt').get_parameter_value().string_value]
     
    
        q_all = traj.rotation.rot.as_quat() #hamiltonian quaternions

        for i in range(traj.n):
            t = traj.t[i]
            p = traj.translation.pos[i]
            v = traj.translation.vel[i]
            a = traj.translation.acc[i]
            w = traj.rotation.angvel[i]
            aa = traj.rotation.angacc[i]
            q = q_all[i]

            messages['paths']['pos'].points.append(rmh.as_point_msg(p))
            messages['paths']['pos'].colors.append(rmh.as_color_msg(colors[int((color_res-1)*magnitudes[i])]))
            messages['index_markers'].append(rmh.as_markerarray_msg(frame_id=traj.id,pos=(p,v,a,w,aa)))
            messages['transforms'].append(rmh.as_transformstamped_msg(t, traj.frame, body_frame, p, q))
            messages['paths']['vel'].poses.append(rmh.as_posestamped_msg(t, traj.frame, v))
            messages['paths']['acc'].poses.append(rmh.as_posestamped_msg(t, traj.frame, a))
            messages['paths']['angvel'].poses.append(rmh.as_posestamped_msg(t, traj.frame, w))
            messages['paths']['angvel'].poses.append(rmh.as_posestamped_msg(t, traj.frame, aa))

        i = 0
        for k,v in features.items():
            if k == 'all':
                pass
            else:
                messages['features'].markers[i].points=[rmh.as_point_msg(v.points['global'][j]) for j in range(v.n)]
                messages['features'].markers[i].colors=[rmh.as_color_msg(v.colors[j],scale=255) for j in range(v.n)]
                i+=1

        return messages

    def start_timers(self, slow_rate=1.0, fast_rate=30.0):
        
        self.t0 = self.get_clock().now().nanoseconds
        fast_group = MutuallyExclusiveCallbackGroup()
        slow_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0/slow_rate, self.publish_paths, callback_group=slow_group)
        self.create_timer(1.0/slow_rate, self.publish_features, callback_group=slow_group)
        self.create_timer(1.0/slow_rate, self.publish_feature_measurements, callback_group=slow_group)
        self.create_timer(1.0/slow_rate, self.publish_camera_infos, callback_group=slow_group)
        
        self.create_timer(1.0/fast_rate, self.index_select, callback_group=fast_group) 
        self.create_timer(1.0/fast_rate, self.broadcast_transform, callback_group=fast_group) 
        self.create_timer(1.0/fast_rate, self.publish_index_markers, callback_group=fast_group) 
        self.create_timer(1.0/fast_rate, self.publish_mark, callback_group=fast_group)
        self.create_timer(1.0/fast_rate, self.publish_index, callback_group=fast_group)

    def index_select(self):
        """
        select the current index in trajectory based on time loop, 
        or based on slider position in gui (not implemented yet)
        """
        if self.playing:
            traj = self.scene.trajectory
            T = traj.dur*1e9 #duration in nanoseconds
            t0 = self.t0 

            t = self.get_clock().now().nanoseconds-t0
            if t>=T: 
                self.t0 = self.get_clock().now().nanoseconds #reset every 'T' nanoseconds
                t = 0
                self.get_logger().info('end of trajectory, looping again...')

            n = traj.n
            i = int((t/T)*n) #index in trajectory
            self.i = i

    def publish_paths(self):
        pub = self.pub['paths']
        msgs = self.messages['paths']
   
        pub['pos'].publish(msgs['pos'])
        pub['vel'].publish(msgs['vel'])
        pub['acc'].publish(msgs['acc'])
        pub['angvel'].publish(msgs['angvel'])
        pub['angacc'].publish(msgs['angacc'])

    def publish_index_markers(self):
        i = self.i
        pub = self.pub['index_markers']
        msg = self.messages['index_markers'][i]
        pub.publish(msg)

    def broadcast_transform(self):
        i = self.i
        now = self.get_clock().now()
        pub = self.pub['transforms']
        msg = self.messages['transforms'][i]
        msg.header.stamp = now.to_msg()
        pub.sendTransform((msg))

    def publish_features(self):
        """
        Publish all of the feature markers
        """
        pub = self.pub['features']
        msg = self.messages['features']
        pub.publish(msg)

    def publish_feature_measurements(self):
        """
        Publish the feature measurements
        """
        pub_measurements = self.pub['feature_measurements']
        msg_measurements = self.messages['feature_measurements']
        pub_measurements.publish(msg_measurements)

        pub_colors = self.pub['feature_colors']
        msg_colors = self.messages['feature_colors']
        pub_colors.publish(msg_colors)

    def publish_camera_infos(self):
        """
        Publish camera info for each camera
        """
        pub = self.pub['camera_infos']
        msgs = self.messages['camera_infos']
        for msg in msgs:
            pub.publish(msg)

    def publish_mark(self):
        pub = self.pub['mark']
        msg = self.messages['mark']
        msg.data = self.increment*self.i
        pub.publish(msg)

    def publish_index(self):
        pub = self.pub['index']
        msg = self.messages['index']
        msg.data = self.i
        pub.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    s = SceneViewer()
    executor = MultiThreadedExecutor()
    executor.add_node(s)
    executor.spin()



   

