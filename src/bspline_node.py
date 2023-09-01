#!/usr/bin/env python 
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node   import Node
import numpy as np
import tf2_ros
from std_msgs.msg import Float32
from builtin_interfaces.msg import Time
from  nav_msgs.msg          import Odometry, Path
from  geometry_msgs.msg     import PolygonStamped, Point32, PoseStamped
from std_srvs.srv           import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation
from bspline_controller import BSplineController
import ros_message_helper as rmh
import threading

"""
Publishes spline
should make this a cpp node if its gonna have to loop through points to turn them into pointstamped messages

poll the bspline controller at a constant rate, convert the arrays to path messages, and the control points to markers

TODO:
functions to publish tf frame
services to start and stop animation

"""
class BSplineNode(Node):
    def __init__(self):
        super().__init__('bspline_node')

        self.controller = BSplineController()
        self.controller_keyboard_start()
    

        display_rate = 3.0# display 'framerate' at which to publish the visualization
        update_rate = 30.0 # rate at which to update the transform to the moving frame

        callback_group_1 = MutuallyExclusiveCallbackGroup()
        callback_group_2 = MutuallyExclusiveCallbackGroup()
        
        self.create_timer(1.0/update_rate, self.publish_transforms, callback_group=callback_group_1) 
        self.create_timer(1.0/display_rate, self.publish_paths, callback_group=callback_group_2)
        self.create_timer(1.0/display_rate, self.publish_markers, callback_group=callback_group_2)

        self.pub_markers = self.create_publisher(MarkerArray,'~/control_pts', 10)
        self.pub_pos = self.create_publisher(Path,'~/pos', 10)
        self.pub_vel = self.create_publisher(Path,'~/vel', 10)
        self.pub_acc = self.create_publisher(Path,'~/acc', 10)
        self.pub_markers = self.create_publisher(MarkerArray,'~/control_pts', 10)
        self.zero_rotation = np.array([0.0,0.0,0.0,1.0])
        self.ready = True
        self.initial_time = self.get_clock().now().nanoseconds
        self.t0 = self.initial_time
        self.t = self.initial_time
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)


    def publish_transforms(self):
        """
        Lookup the position and all derivative values at the current time.
        Publish the transform to the moving frame, print velocity and acceleration magnitudes.
        When the end of the spline is reached, loop back to the beginning.
        """
        now = self.get_clock().now()

        pos = self.controller.pos_bspline
        vel = self.controller.vel_bspline
        acc = self.controller.acc_bspline
        total_dur_nanosec = (len(self.controller.control_pts)-self.controller.order)*1e9 #total duration of the bslpline in nanoseconds

        n = len(pos)
        T = total_dur_nanosec
        t0 = self.t0

        t = self.get_clock().now().nanoseconds-t0
        if t>=T: #reset
            self.t0 = self.get_clock().now().nanoseconds
            t = 0
        
        index = int((t/T)*n)
        p=pos[index]
        v=vel[index]
        a=acc[index]
        msg_tf = rmh.as_transformstamped_msg(now.to_msg(),'global','imu', pos[index], self.zero_rotation)
        self.tf_broadcaster.sendTransform(msg_tf)

        self.get_logger().info('t: {:.02f}  pos: {:.02f},{:.02f},{:.02f}  vel: {:.02f}  acc: {:.02f}'.format(
              t*1e-9,*p,np.linalg.norm(v),np.linalg.norm(a)))
        
        self.t = t

    def publish_markers(self):
        """
        Publish markers for each control point
        """
        markers = MarkerArray()
        for i in range(len(self.controller.control_pts)):
            m = rmh.as_marker_msg(rmh.as_stamp(0.0),'global',self.controller.control_pts[i,:],self.zero_rotation)
            if i==self.controller.selection:
                # m.color = rmh.as_color('green')
                m.color.a = 1.0
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 1.0
            m.id = i
            markers.markers.append(m)
        self.pub_markers.publish(markers)
        
    
    def publish_paths(self):

        pos = self.controller.pos_bspline
        vel = self.controller.vel_bspline
        acc = self.controller.acc_bspline
        if pos is not None:
            pos_path = self.as_path_msg(rmh.as_stamp(0.0),'global',self.controller.pos_bspline) #TODO: cython
            vel_path = self.as_path_msg(rmh.as_stamp(0.0),'global',self.controller.vel_bspline)
            acc_path = self.as_path_msg(rmh.as_stamp(0.0),'global',self.controller.acc_bspline)

            self.pub_pos.publish(pos_path)
            self.pub_vel.publish(vel_path)
            self.pub_acc.publish(acc_path)
        else:
            print('pos is none')
        
    def as_path_msg(self,stamp,frame_id,poses):
        p = Path()
        p.header.stamp=stamp
        p.header.frame_id = frame_id
        # p.poses=[as_posestamped_msg(as_stamp(0.0),'global',poses[i,:],self.zero_rotation) for i in range(len(poses))]
        p.poses=[rmh.as_posestamped_msg(rmh.as_stamp(0.0),'global',poses[i,:],self.zero_rotation) for i in range(len(poses))]
        return p
    
    def controller_keyboard_start(self):
        thread = threading.Thread(target=self.controller.keyboard_start(), daemon=True)
        thread.start()
        # thread.join()
   
if __name__ == '__main__':
    rclpy.init()
    executor = MultiThreadedExecutor()
    b = BSplineNode()
    executor.add_node(b)
    executor.spin()
    