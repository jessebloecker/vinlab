#!/usr/bin/env python 
import sys
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node   import Node
import numpy as np
import tf2_ros
from  nav_msgs.msg    import Path
from visualization_msgs.msg import  MarkerArray
import ros_message_helper as rmh
from functools import partial
from trajectory import Trajectory
import yaml

"""
Load trajectory from yaml config file
Loop through the trajectory and publish transforms, paths, and other markers
"""
class TrajectoryPlayer(Node):
    def __init__(self,config):
        super().__init__('trajectory_player')
        self.trajectory = self.load_trajectory(config)
        p,s = self.init_communication()
        self._publishers = p
        self._subscribers = s
        self._messages = self.get_messages()
        self.playing = True
        self.start_timers()
        
    def load_trajectory(self,config_file):
        # parse yaml config
        config = yaml.load(open(config_file),Loader=yaml.FullLoader)
        _trajectory = config['trajectory']
        return Trajectory.config(_trajectory)
    
    def init_communication(self):
        """
        Initialize publishers, subscribers, 
        """
        traj = self.trajectory
        global_frame = traj.frame
        body_frame = traj.body_frame
        now = self.get_clock().now().nanoseconds
        
        publishers = {}
        subscribers = {}
        publishers['index_markers'] = self.create_publisher(MarkerArray,'~/index_markers', 10)
        publishers['transforms'] = tf2_ros.TransformBroadcaster(self)
        publishers['paths'] = []
       
        for i in ['pos','vel','acc']:
            publishers['paths'].append(self.create_publisher(Path,'~/'+traj.name+'/'+i,10))
            subscribers[i] = self.create_subscription(Path,'/input/'+i , partial(self.update_trajectory, i),10)

        return publishers, subscribers
    
    def get_messages(self):
        """
        Convert trajectory to Path messages, list of transforms, and list of marker arrays
        corresponding to each time step in the trajectory
        """
        messages = {'paths':[Path(),Path(),Path()],'index_markers':[],'transforms':[]}
        messages['paths'][0].header.frame_id = self.trajectory.frame
        messages['paths'][1].header.frame_id = self.trajectory.frame
        messages['paths'][2].header.frame_id = self.trajectory.frame
        
        traj = self.trajectory
        global_frame = traj.frame
        body_frame = traj.body_frame

        no_rot = np.array([0.,0.,0.,1.])
        pva = np.zeros((3,3))

        for i in range(traj.n):
            t = traj.t[i]
            p = traj.translation.pos[i]
            v = traj.translation.vel[i]
            a = traj.translation.acc[i]
            q = traj.rotation.q[i]
            pva[:,0], pva[:,1], pva[:,2] = p,v,a


            messages['index_markers'].append(rmh.as_markerarray_msg(t, global_frame, pva.T))
            messages['transforms'].append(rmh.as_transformstamped_msg(t, global_frame, body_frame, p, q))
            messages['paths'][0].poses.append(rmh.as_posestamped_msg(t, global_frame, p, q))
            messages['paths'][1].poses.append(rmh.as_posestamped_msg(t, global_frame, v, no_rot))
            messages['paths'][2].poses.append(rmh.as_posestamped_msg(t, global_frame, a, no_rot))
        return messages


    def start_timers(self, slow_rate=1.0, fast_rate=30.0):

        self.t0 = self.get_clock().now().nanoseconds

        g1 = MutuallyExclusiveCallbackGroup()
        g2 = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0/fast_rate, self.index_select, callback_group=g1) 
        self.create_timer(1.0/fast_rate, self.broadcast_transform, callback_group=g1) 
        self.create_timer(1.0/fast_rate, self.publish_index_markers, callback_group=g1) 
        self.create_timer(1.0/slow_rate, self.publish_paths, callback_group=g2)
        self.create_timer(1.0/slow_rate, self.publish_index_markers, callback_group=g2)
    
    def update_trajectory(self, msg, name):
        """
        Subscriber callback:
        replace current Path message with received Path message
        then update the trajectory, recompute rotations

        TODO: handle rotation part
        """
        if name == 'pos':
            self._messages['pos'] = msg
            self.trajectory.translation.pos = rmh.as_ndarray(msg)
        elif name == 'vel':
            self._messages['vel'] = msg
            self.trajectory.translation.vel = rmh.as_ndarray(msg)
        elif name == 'acc':
            self._messages['acc'] = msg
            self.trajectory.translation.acc = rmh.as_ndarray(msg)
        else:
            raise ValueError('Invalid name: {}'.format(name))
        
    def index_select(self):
        """
        select the current index in trajectory based on time loop, 
        or based on slider position in gui (not implemented yet)
        """
        if self.playing:
            traj = self.trajectory
            T = traj.dur*1e9 #duration in nanoseconds
            t0 = self.t0 #reset every 'T' nanoseconds

            t = self.get_clock().now().nanoseconds-t0
            if t>=T: 
                self.t0 = self.get_clock().now().nanoseconds 
                t = 0

            n = traj.n
            i = int((t/T)*n) #index in trajectory
            self.i = i
        else:
            pass

    def broadcast_transform(self):
        i = self.i
        pub = self._publishers['transforms']
        msg = self._messages['transforms'][i]
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        pub.sendTransform(msg)

    def publish_paths(self):
        """
        Publish all of the Path messages 
        """
        pub = self._publishers['paths']
        msgs = self._messages['paths']
        
        for j in range(3):
            pub[j].publish(msgs[j])

    def publish_index_markers(self):
        i = self.i
        pub = self._publishers['index_markers']
        msg = self._messages['index_markers'][i]
        pub.publish(msg)
    
if __name__ == '__main__':
    rclpy.init()
    executor = MultiThreadedExecutor()
    if len(sys.argv) < 2:
        print('provide config file \n\n usage: ros2 run motion_tools trajectory_player.py <config_file>')
        exit() 
    config=sys.argv[1]
    t = TrajectoryPlayer(config)
    executor.add_node(t)
    executor.spin()

