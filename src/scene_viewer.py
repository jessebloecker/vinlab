#!/usr/bin/env python 
import sys
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import numpy as np
import tf2_ros
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, Float32, Int16, String
from nav_msgs.msg    import Path
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import  MarkerArray, Marker
import ros_message_helper as rmh
from functools import partial
from trajectory import Trajectory
from scene import Scene
import yaml
from scipy.spatial.transform import Rotation
from geometry_utils import linear_interp
from color_helper import ColorHelper
from std_msgs.msg  import *
from array_utils import ValueMatcher

np.set_printoptions(threshold=sys.maxsize,suppress=True,precision=3,linewidth=100)

class SceneViewer(Node):
    def __init__(self):
        super().__init__('scene_viewer')
        self.declare_params()
        scene_config =  self.get_parameter('scene').get_parameter_value().string_value

        if not scene_config:
            self.get_logger().error('no scene configuration set - provide path to yaml file. \n   usage: \n   '
                                    'ros2 run vinlab scene_viewer.py --ros-args --params-file /home/jesse/ros2_ws/src/vinlab/config/scene_viewer.yaml')
            sys.exit()
        else:
            self.get_logger().info('loading scene configuration: {}'.format(scene_config))
        self.scene = Scene.config(scene_config)
        self.scene.print_scene_info()

        tg = self.scene.trajectory_group
        ref = tg.reference
        main = tg.main
        self.traj_ref = tg.trajectories[ref]
   
        self.traj_main = tg.trajectories[main]
        matcher = ValueMatcher(self.traj_ref.times, self.traj_main.times,tol=2.0) #this should be an attribute of the trajectory group my dudes
        self.cors_a, self.cors_b = matcher.cors

        p,s = self.init_pub_sub()
        self.pub = p
        self.sub = s
        self.messages = self.get_messages()

        # self.playing = False
        self.playing = True
        self.playback_speed = self.get_parameter('playback_speed').get_parameter_value().double_value
        slow_rate = self.get_parameter('slow_rate').get_parameter_value().double_value
        fast_rate = self.get_parameter('fast_rate').get_parameter_value().double_value
        self.start_timers(slow_rate,fast_rate)

        self.i = 0
        self.j = 0
        self.increment = float(1/self.traj_ref.n)
        self.viewer_initial_time = self.get_clock().now().nanoseconds
        self.t0 = 0.0

    def declare_params(self):
        self.declare_parameter('scene','')
        self.declare_parameter('loop',False)
        self.declare_parameter('playback_speed',1.0)
        self.declare_parameter('slow_rate',1.0)
        self.declare_parameter('fast_rate',30.0)
        self.declare_parameter('line_width',0.01)
        self.declare_parameter('control_point_size',0.03)
        self.declare_parameter('feature_point_size',0.05)
        self.declare_parameter('main_trajectory.id','estimate')
        self.declare_parameter('main_trajectory.colors.res',100)
        self.declare_parameter('main_trajectory.colors.wrt','vel')
        self.declare_parameter('main_trajectory.colors.color_min','green')
        self.declare_parameter('main_trajectory.colors.color_max','green')
    
    def init_pub_sub(self):
        """
        initialize publishers and subscribers
        """
        traj_main = self.traj_main
        now = self.get_clock().now().nanoseconds
        
        sub_slider = self.create_subscription(Float32,'/slider/value',self.slider_cb,10)
        subscribers = {}
        publishers = {}
        publishers['index_markers'] = self.create_publisher(MarkerArray,'~/index_markers', 10)
        publishers['transforms'] = tf2_ros.TransformBroadcaster(self)
        publishers['static_transforms'] = tf2_ros.StaticTransformBroadcaster(self)
        publishers['features'] = self.create_publisher(MarkerArray,'~/features',10)
        publishers['feature_measurements'] = self.create_publisher(Int16MultiArray,'~/feature_measurements',10)
        publishers['feature_colors'] = self.create_publisher(Int16MultiArray,'~/feature_colors',10)
        publishers['camera_infos'] = self.create_publisher(CameraInfo,'~/camera_info',10)
        publishers['mark'] = self.create_publisher(Float32,'~/mark',10)
        publishers['index'] = self.create_publisher(Int16,'~/index',10)
        publishers['control_points']= {'ref':  self.create_publisher(Marker,'~/ref/control_points',10),
                                       'main': self.create_publisher(Marker,'~/main/control_points',10)}
        publishers['paths'] = {'pos_main': self.create_publisher(Marker,'~/main/pos',10)}
        publishers['output_config'] = self.create_publisher(String, '~/output_config', 10)
        publishers['reprojections'] = self.create_publisher(Marker,'~/reprojections',10)
        for i in ('pos','vel','acc', 'angvel', 'angacc'):
            publishers['paths'][i+'_ref'] = self.create_publisher(Path,'~/ref/'+i,10)
        return publishers, subscribers
    
    def get_messages(self):
        """
        Convert trajectory to Path messages, list of transforms, and list of marker arrays
        corresponding to each time step in the trajectory
        """
        traj_ref = self.traj_ref
        cam_id = [i for i in self.scene.platform.sensors.keys() if self.scene.platform.sensors[i].type=='camera'][0]
        n,m = self.scene.measurements[cam_id].values.shape[0:2]
        # print('number of measurements: {}'.format(self.scene.measurements[cam_id].values.shape))
        traj_main = self.traj_main
        base_frame = self.scene.platform.base_frame
        global_frame = 'global' #keep these fixed until dynamically change the rviz config
        body_frame = base_frame

        features = self.scene.features
        meas = self.scene.measurements[cam_id].values.swapaxes(1,2).reshape(2*n,m).astype(np.int16)
        assert meas.shape == (2*n,m)
        sensors = self.scene.platform.sensors

        line_width = self.get_parameter('line_width').get_parameter_value().double_value
        control_point_size = self.get_parameter('control_point_size').get_parameter_value().double_value
        feature_point_size = self.get_parameter('feature_point_size').get_parameter_value().double_value
        messages = {'paths':{'pos_main': rmh.as_marker_msg(frame_id=global_frame, scale=[line_width]*3, marker_type=4),
                             'pos_ref': rmh.as_path_msg(frame_id=global_frame),
                             'vel_ref': rmh.as_path_msg(frame_id=global_frame), 
                             'acc_ref': rmh.as_path_msg(frame_id=global_frame),
                             'angvel_ref': rmh.as_path_msg(frame_id=global_frame),
                             'angacc_ref': rmh.as_path_msg(frame_id=global_frame)},
                    'control_points': {'ref': rmh.as_marker_msg(frame_id=global_frame, scale=[control_point_size]*3, marker_type=7),
                                       'main': rmh.as_marker_msg(frame_id=global_frame, scale=[control_point_size]*3, marker_type=7)},
                    'index_markers': [],
                    'transforms':{'ref':[], 'main':[]}, 
                    'static_transforms':[],
                    'mark': Float32(),
                    'index': Int16(),
                    'features':rmh.as_markerarray_msg(frame_id=global_frame,n=len(features),marker_type=7, scale=[feature_point_size]*3),
                    'reprojections': rmh.as_marker_msg(frame_id='cam', scale=[feature_point_size]*3, marker_type=7, color=(0.,1.,0.,1.)),
                    # 'reprojections': rmh.as_marker_msg(frame_id='global', scale=[0.1]*3, marker_type=7, color=(0.,1.,0.,1.)),
                    'camera_infos': [] ,
                    'output_config': String(data=self.scene.output_config)}
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
        messages['feature_measurements'] = measurements_msg #todo make this a list of messages for all cameras

        feature_colors = features['all'].colors
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
            if not k==base_frame:
                static_tf_msg = rmh.as_transformstamped_msg(0, base_frame, k, v.pos, v.rot.as_quat())
                messages['static_transforms'].append(static_tf_msg)

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

        if self.scene.static_frames is not None:
            for k,v in self.scene.static_frames.items():
                # print('APPENDING STATIC TRANSFORM: {} -> {}'.format(k,v))
                messages['static_transforms'].append(rmh.as_transformstamped_msg(0, global_frame, v.id, v.pos, v.rot.as_quat()))


        ch = ColorHelper('rgba',scale=1)
        color_res = self.get_parameter('main_trajectory.colors.res').get_parameter_value().integer_value
        color_min = ch.__dict__[(self.get_parameter('main_trajectory.colors.color_min').get_parameter_value().string_value).upper()]
        color_max = ch.__dict__[(self.get_parameter('main_trajectory.colors.color_max').get_parameter_value().string_value).upper()]
        colors = linear_interp(color_min, color_max, color_res)
        
        color_map_wrt = self.get_parameter('main_trajectory.colors.wrt').get_parameter_value().string_value
        if color_map_wrt in ['pos', 'vel', 'acc']:
            magnitudes = traj_main.translation.__dict__[color_map_wrt].normalized_norms
        else:
            self.get_logger().warn('trajectory color with respect to unknown quantity: \'{}\', ignoring...'.format(color_map_wrt))
            magnitudes = np.zeros(traj_main.n)
     
        q_all = traj_ref.rotation.rot.as_quat() #hamiltonian quaternions
        # q_all_main = traj_main.rotation.rot.as_quat() #hamiltonian quaternions
        for i in range(traj_ref.n): #fill messages corresponding to the reference trajectory
            t = traj_ref.times[i]
            p = traj_ref.translation.pos.values[i]
            if i==0: 
                print('first ref position: {}'.format(p))
            v = traj_ref.translation.vel.values[i]
            a = traj_ref.translation.acc.values[i]
            w = traj_ref.rotation.angvel.values[i]
            aa = traj_ref.rotation.angacc.values[i]
            q = q_all[i]

            messages['index_markers'].append(rmh.as_markerarray_msg(frame_id=global_frame,pos=(p,v,a,w,aa)))
            messages['paths']['pos_ref'].poses.append(rmh.as_posestamped_msg(t, global_frame, p))
            messages['paths']['vel_ref'].poses.append(rmh.as_posestamped_msg(t, global_frame, v))
            messages['paths']['acc_ref'].poses.append(rmh.as_posestamped_msg(t, global_frame, a))
            messages['paths']['angvel_ref'].poses.append(rmh.as_posestamped_msg(t, global_frame, w))
            messages['paths']['angvel_ref'].poses.append(rmh.as_posestamped_msg(t, global_frame, aa))
            messages['transforms']['ref'].append(rmh.as_transformstamped_msg(t, global_frame, body_frame, p, q)) #THIS MIGHT BE WRONG
        if traj_ref.translation.type=='bspline':
            pts = traj_ref.translation.control_points
            for i in range(len(pts)):
                messages['control_points']['ref'].points.append(rmh.as_point_msg(pts[i]))
                messages['control_points']['ref'].colors.append(rmh.as_color_msg(ch.WHITE))

    
        for i in range(traj_main.n): # fill messages corresponding to the main trajectory
            p_main = traj_main.translation.pos.values[i]
            messages['paths']['pos_main'].points.append(rmh.as_point_msg(p_main))
            messages['paths']['pos_main'].colors.append(rmh.as_color_msg(colors[int((color_res-1)*magnitudes[i])]))
       
        i = 0
        for k,v in features.items():
            if k == 'all':
                pass
            else:
                messages['features'].markers[i].points=[rmh.as_point_msg(v.points['global'][j]) for j in range(v.n)]
                messages['features'].markers[i].colors=[rmh.as_color_msg(v.colors[j],scale=255,alpha=0.3) for j in range(v.n)]
                i+=1
        
        return messages

    def start_timers(self, slow_rate=1.0, fast_rate=30.0):
        fast_group = MutuallyExclusiveCallbackGroup()
        slow_group = MutuallyExclusiveCallbackGroup()

        #make a dictionary of callbacks by topic, create timer for each one
        self.create_timer(1.0/slow_rate, self.publish_paths, callback_group=slow_group)
        self.create_timer(1.0/slow_rate, self.publish_control_points, callback_group=slow_group)
        self.create_timer(1.0/slow_rate, self.publish_features, callback_group=slow_group)
        self.create_timer(1.0/slow_rate, self.publish_feature_measurements, callback_group=slow_group)
        self.create_timer(1.0/slow_rate, self.publish_camera_infos, callback_group=slow_group)
        self.create_timer(1.0/slow_rate, self.broadcast_static_transforms, callback_group=slow_group)

        self.create_timer(1.0/fast_rate, self.index_select, callback_group=fast_group) 
        self.create_timer(1.0/fast_rate, self.broadcast_transform, callback_group=fast_group) 
        self.create_timer(1.0/fast_rate, self.publish_index_markers, callback_group=fast_group) 
        self.create_timer(1.0/fast_rate, self.publish_mark, callback_group=fast_group)
        self.create_timer(1.0/fast_rate, self.publish_index, callback_group=fast_group)
        # self.create_timer(1.0/fast_rate, self.publish_reprojections, callback_group=fast_group)

    def index_select(self):
        """
        select the current index in trajectory based on time loop, 
        or based on slider position in gui (not implemented yet)
        """
        s = self.playback_speed
        dur = self.traj_ref.dur
        loop_count = 0
        if self.playing:
            elapsed = self.get_clock().now().nanoseconds - self.viewer_initial_time #time elapsed since viewer started
            T = dur*1e9 #duration in nanoseconds
            ref_t0 = 0.0 #reference trajectory always starts at 0
            t = s*(elapsed-self.t0) #t goes from 0 to duration and then resets to 0, t0 incremets by dur each reset
            self.t = t #maybe not needed
            if t>=T: 
                self.t0 = self.get_clock().now().nanoseconds - self.viewer_initial_time #reset every 'T' nanoseconds
                t = 0.0
                loop_count += 1

            n_ref = self.traj_ref.n
            n_main = self.traj_main.n
            i = int((t/T)*n_ref) #index in trajectory
            self.i = i

            if self.cors_a[i] >= 0:
                self.j = self.cors_a[i]
            print('playback speed: {} time: {:.3f}/{:.3f}s index(ref): {}/{}  index(main) {}/{} loop count: {}'.format(s,t*1e-9, dur, i, n_ref, self.j, n_main, loop_count))

    def slider_cb(self,msg):
        n = self.scene.trajectory.n
        self.i = int(msg.data*(n-1))
        self.get_logger().info('slider value: {}, index: {}'.format(msg.data,self.i))

    def publish_paths(self):
        pub = self.pub['paths']
        msgs = self.messages['paths']
   
        pub['pos_main'].publish(msgs['pos_main'])
        pub['pos_ref'].publish(msgs['pos_ref'])
        pub['vel_ref'].publish(msgs['vel_ref'])
        pub['acc_ref'].publish(msgs['acc_ref'])
        pub['angvel_ref'].publish(msgs['angvel_ref'])
        pub['angacc_ref'].publish(msgs['angacc_ref'])

    def publish_control_points(self):
        pub = self.pub['control_points']
        msgs = self.messages['control_points']
        pub['ref'].publish(msgs['ref'])
        pub['main'].publish(msgs['main'])

    def publish_index_markers(self):
        i = self.i
        pub = self.pub['index_markers']
        msg = self.messages['index_markers'][i]
        pub.publish(msg)

    def broadcast_transform(self):
        i = self.i
        j = self.j
        now = self.get_clock().now()
        pub = self.pub['transforms']
        msg_ref = self.messages['transforms']['ref'][i]
        msg_ref.header.stamp = now.to_msg()
        pub.sendTransform(msg_ref)

    def broadcast_static_transforms(self):
        pub = self.pub['static_transforms']
        msgs = self.messages['static_transforms']
        for msg in msgs:
            pub.sendTransform(msg)

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

